#include "radar_obstacle_avoidance/rtk_gps_parser.h"
#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <fstream>

using namespace radar_obstacle_avoidance;
RTKGPSParser::RTKGPSParser() :
    running_(true),
    pps_synchronized_(false),
    fix_status_(0),
    hdop_(99.9),
    satellite_count_(0),
    event_pending_(false)
{
    nh_.param("serial_port", port_, std::string("/dev/ttyACM0"));
    nh_.param("baud_rate", baud_rate_, 115200);
    nh_.param("enable_pps", enable_pps_, true);
    nh_.param("pps_device", pps_device_, std::string("/dev/pps0"));

    rtk_pub_ = nh_.advertise<RTKData>("/rtk/data", 10);
    navsat_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/rtk/fix", 10);
    event_pub_ = nh_.advertise<RTKEvent>("/rtk/event", 10);
    time_ref_pub_ = nh_.advertise<sensor_msgs::TimeReference>("/rtk/time_ref", 10);
    pps_pub_ = nh_.advertise<std_msgs::Bool>("/rtk/pps", 10);

    feature_event_sub_ = nh_.subscribe<std_msgs::Bool>(
        "/feature_rich_event", 1, &RTKGPSParser::featureEventCallback, this);

    if (!initSerial()) {
        ROS_ERROR("串口初始化失败");
        return;
    }

    ROS_INFO("RTK-GPS 解析器初始化完成，端口: %s", port_.c_str());
}

RTKGPSParser::~RTKGPSParser() {
    running_ = false;
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
}

bool RTKGPSParser::initSerial() {
    try {
        serial_port_ = std::make_unique<serial::Serial>(
            port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
        
        if (!serial_port_->isOpen()) {
            ROS_ERROR("无法打开串口 %s", port_.c_str());
            return false;
        }
        
        serial_port_->setRTS(false);
        serial_port_->setDTR(false);
        
        ROS_INFO("串口 %s 已打开，波特率 %d", port_.c_str(), baud_rate_);
        
        serial_thread_ = std::thread(&RTKGPSParser::serialReadThread, this);
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("串口错误: %s", e.what());
        return false;
    }
}

void RTKGPSParser::serialReadThread() {
    std::string buffer;
    while (running_ && ros::ok()) {
        try {
            if (serial_port_->available()) {
                std::string data = serial_port_->read(serial_port_->available());
                buffer += data;
                
                size_t pos;
                while ((pos = buffer.find('\n')) != std::string::npos) {
                    std::string line = buffer.substr(0, pos);
                    buffer.erase(0, pos + 1);
                    
                    if (!line.empty() && line.back() == '\r') {
                        line.pop_back();
                    }
                    
                    if (!line.empty() && line[0] == '$') {
                        processNMEASentence(line);
                    }
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "串口读取错误: %s", e.what());
        }
    }
}

void RTKGPSParser::processNMEASentence(const std::string& sentence) {
    if (!validateChecksum(sentence)) {
        ROS_DEBUG_THROTTLE(5.0, "校验和错误: %s", sentence.c_str());
        return;
    }
    
    std::vector<std::string> fields = splitString(sentence, ',');
    if (fields.empty()) return;
    
    std::string sentence_type = fields[0].substr(1);
    bool parsed = false;
    
    if (sentence_type.find("GGA") != std::string::npos) {
        parsed = parseGGA(fields);
    } else if (sentence_type.find("RMC") != std::string::npos) {
        parsed = parseRMC(fields);
    } else if (sentence_type.find("GSA") != std::string::npos) {
        parsed = parseGSA(fields);
    } else if (sentence_type.find("VTG") != std::string::npos) {
        parsed = parseVTG(fields);
    } else if (sentence_type.find("ZDA") != std::string::npos) {
        parsed = parseZDA(fields);
    } else if (sentence_type.find("GST") != std::string::npos) {
        parsed = parseGST(fields);
    }
    
    if (parsed) {
        publishRTKData();
        publishNavSatFix();
        publishTimeReference();
        
        if (event_pending_ && fix_status_ >= 4) {
            pending_event_.event_triggered = true;
            pending_event_.position_accuracy = current_rtk_data_.h_accuracy;
            event_pub_.publish(pending_event_);
            
            ROS_WARN("RTK 事件记录: 纬度=%.6f, 经度=%.6f, 海拔=%.2fm, 航向=%.1f°",
                    pending_event_.latitude, pending_event_.longitude,
                    pending_event_.altitude, pending_event_.heading);
            event_pending_ = false;
        }
    }
}

bool RTKGPSParser::parseGGA(const std::vector<std::string>& fields) {
    if (fields.size() < 15) return false;
    
    try {
        // 解析时间
        if (!fields[1].empty()) {
            std::string time_str = fields[1];
            std::string date_str = fields.size() > 9 ? fields[9] : "";
            if (date_str.empty() && !last_gps_time_.isZero()) {
                date_str = "010100"; // 默认日期
            }
            current_rtk_data_.gps_time = parseGPSTime(time_str, date_str);
        }
        
        // 解析纬度
        if (!fields[2].empty() && !fields[3].empty()) {
            current_rtk_data_.latitude = parseCoordinate(fields[2], fields[3]);
        }
        
        // 解析经度
        if (!fields[4].empty() && !fields[5].empty()) {
            current_rtk_data_.longitude = parseCoordinate(fields[4], fields[5]);
        }
        
        // 解析定位质量
        if (!fields[6].empty()) {
            int fix_quality = std::stoi(fields[6]);
            switch (fix_quality) {
                case 0: fix_status_ = 0; break; // 无效
                case 1: fix_status_ = 1; break; // GPS定位
                case 2: fix_status_ = 2; break; // 差分GPS
                case 4: fix_status_ = 4; break; // RTK固定解
                case 5: fix_status_ = 5; break; // RTK浮点解
                default: fix_status_ = 0; break;
            }
            current_rtk_data_.fix_status = fix_status_;
        }
        
        // 解析卫星数量
        if (!fields[7].empty()) {
            satellite_count_ = std::stoi(fields[7]);
            current_rtk_data_.sat_num = satellite_count_;
        }
        
        // 解析HDOP
        if (!fields[8].empty()) {
            hdop_ = std::stof(fields[8]);
            current_rtk_data_.hdop = hdop_;
            current_rtk_data_.h_accuracy = hdop_ * 2.5;
        }
        
        // 解析海拔高度
        if (!fields[9].empty()) {
            current_rtk_data_.altitude = std::stof(fields[9]);
        }
        
        // 解析大地水准面高度
        if (!fields[11].empty()) {
            current_rtk_data_.ellipsoidal_height = std::stof(fields[11]);
        }
        
        return true;
    } catch (const std::exception& e) {
        ROS_DEBUG("GGA 解析错误: %s", e.what());
        return false;
    }
}

bool RTKGPSParser::parseRMC(const std::vector<std::string>& fields) {
    if (fields.size() < 12) return false;
    
    try {
        // 解析时间
        if (!fields[1].empty() && !fields[9].empty()) {
            std::string time_str = fields[1];
            std::string date_str = fields[9];
            current_rtk_data_.gps_time = parseGPSTime(time_str, date_str);
        }
        
        // 解析状态
        if (!fields[2].empty()) {
            char status = fields[2][0];
            if (status == 'A') {
                // 有效定位
                if (fix_status_ == 0) fix_status_ = 1;
            } else if (status == 'V') {
                fix_status_ = 0; // 无效定位
            }
        }
        
        // 解析纬度
        if (!fields[3].empty() && !fields[4].empty()) {
            current_rtk_data_.latitude = parseCoordinate(fields[3], fields[4]);
        }
        
        // 解析经度
        if (!fields[5].empty() && !fields[6].empty()) {
            current_rtk_data_.longitude = parseCoordinate(fields[5], fields[6]);
        }
        
        // 解析速度
        if (!fields[7].empty()) {
            current_rtk_data_.speed = std::stof(fields[7]) * 1.852; // 节转km/h
        }
        
        // 解析航向
        if (!fields[8].empty()) {
            current_rtk_data_.heading = std::stof(fields[8]);
        }
        
        // 解析日期
        if (!fields[9].empty()) {
            std::string date_str = fields[9];
            if (date_str.length() == 6) {
                int day = std::stoi(date_str.substr(0, 2));
                int month = std::stoi(date_str.substr(2, 2));
                int year = 2000 + std::stoi(date_str.substr(4, 2));
                // 日期信息已用于时间解析，这里可以记录周数
                // 简单的周数计算
                struct tm timeinfo = {};
                timeinfo.tm_mday = day;
                timeinfo.tm_mon = month - 1;
                timeinfo.tm_year = year - 1900;
                time_t t = mktime(&timeinfo);
                current_rtk_data_.week_number = static_cast<uint32_t>(t / (7 * 24 * 3600));
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        ROS_DEBUG("RMC 解析错误: %s", e.what());
        return false;
    }
}

bool RTKGPSParser::parseGSA(const std::vector<std::string>& fields) {
    if (fields.size() < 18) return false;
    
    try {
        // 解析定位模式
        if (!fields[1].empty()) {
            char mode = fields[1][0];
            if (mode == 'M') {
                // 手动模式
            } else if (mode == 'A') {
                // 自动模式
            }
        }
        
        // 解析定位类型
        if (!fields[2].empty()) {
            int fix_type = std::stoi(fields[2]);
            if (fix_type == 1) {
                // 无定位
                if (fix_status_ < 1) fix_status_ = 0;
            } else if (fix_type == 2) {
                // 2D定位
                if (fix_status_ < 2) fix_status_ = 1;
            } else if (fix_type == 3) {
                // 3D定位
                if (fix_status_ < 3) fix_status_ = 2;
            }
        }
        
        // 解析PDOP
        if (!fields[15].empty()) {
            current_rtk_data_.pdop = std::stof(fields[15]);
        }
        
        // 解析HDOP
        if (!fields[16].empty()) {
            float new_hdop = std::stof(fields[16]);
            if (new_hdop > 0) {
                hdop_ = new_hdop;
                current_rtk_data_.hdop = hdop_;
                current_rtk_data_.h_accuracy = hdop_ * 2.5;
            }
        }
        
        // 解析VDOP
        if (!fields[17].empty()) {
            current_rtk_data_.vdop = std::stof(fields[17]);
            current_rtk_data_.v_accuracy = current_rtk_data_.vdop * 2.5;
        }
        
        return true;
    } catch (const std::exception& e) {
        ROS_DEBUG("GSA 解析错误: %s", e.what());
        return false;
    }
}

bool RTKGPSParser::parseVTG(const std::vector<std::string>& fields) {
    if (fields.size() < 9) return false;
    
    try {
        // 解析真北航向
        if (!fields[1].empty()) {
            float heading_true = std::stof(fields[1]);
            current_rtk_data_.heading = heading_true;
        }
        
        // 解析磁北航向
        if (!fields[3].empty()) {
            // 可选的磁北航向
        }
        
        // 解析速度(节)
        if (!fields[5].empty()) {
            float speed_knots = std::stof(fields[5]);
            current_rtk_data_.speed = speed_knots * 1.852; // 转km/h
        }
        
        // 解析速度(km/h)
        if (!fields[7].empty()) {
            float speed_kmh = std::stof(fields[7]);
            // 如果存在，使用km/h速度
            current_rtk_data_.speed = speed_kmh;
        }
        
        return true;
    } catch (const std::exception& e) {
        ROS_DEBUG("VTG 解析错误: %s", e.what());
        return false;
    }
}

bool RTKGPSParser::parseZDA(const std::vector<std::string>& fields) {
    if (fields.size() < 7) return false;
    
    try {
        // 解析时间
        if (!fields[1].empty()) {
            std::string time_str = fields[1];
            std::string date_str;
            
            // 从ZDA消息中构建日期
            if (!fields[2].empty() && !fields[3].empty() && !fields[4].empty()) {
                int day = std::stoi(fields[2]);
                int month = std::stoi(fields[3]);
                int year = std::stoi(fields[4]);
                
                std::stringstream ss;
                ss << std::setw(2) << std::setfill('0') << day
                   << std::setw(2) << std::setfill('0') << month
                   << std::setw(2) << std::setfill('0') << (year % 100);
                date_str = ss.str();
            }
            
            if (!date_str.empty()) {
                current_rtk_data_.gps_time = parseGPSTime(time_str, date_str);
            }
        }
        
        // 解析时区信息
        if (fields.size() > 5 && !fields[5].empty() && !fields[6].empty()) {
            // 时区和夏令时信息，可选
        }
        
        return true;
    } catch (const std::exception& e) {
        ROS_DEBUG("ZDA 解析错误: %s", e.what());
        return false;
    }
}

bool RTKGPSParser::parseGST(const std::vector<std::string>& fields) {
    if (fields.size() < 9) return false;
    
    try {
        // 解析时间
        if (!fields[1].empty()) {
            // GST消息的时间通常与GGA一致，这里不重复解析
        }
        
        // 解析RMS误差
        if (!fields[2].empty()) {
            float rms = std::stof(fields[2]);
            // RMS误差可作为精度参考
        }
        
        // 解析标准差
        if (!fields[3].empty()) {
            current_rtk_data_.h_accuracy = std::stof(fields[3]) * 2.0; // 1-sigma转2-sigma
        }
        
        if (!fields[4].empty()) {
            current_rtk_data_.v_accuracy = std::stof(fields[4]) * 2.0;
        }
        
        // 解析协方差
        if (fields.size() > 8 && !fields[8].empty()) {
            float heading_error = std::stof(fields[8]);
            current_rtk_data_.heading_accuracy = heading_error;
        }
        
        return true;
    } catch (const std::exception& e) {
        ROS_DEBUG("GST 解析错误: %s", e.what());
        return false;
    }
}

void RTKGPSParser::publishRTKData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    current_rtk_data_.header.stamp = ros::Time::now();
    current_rtk_data_.header.frame_id = "rtk_gps";
    current_rtk_data_.pps_sync = pps_synchronized_;
    
    // 更新PPS偏移
    if (pps_synchronized_ && !pps_queue_.empty()) {
        ros::Time pps_time = pps_queue_.front();
        pps_queue_.pop();
        current_rtk_data_.pps_offset = (ros::Time::now() - pps_time).toSec();
        
        // PPS同步精度监控
        if (std::abs(current_rtk_data_.pps_offset) > 0.001) {
            ROS_WARN_THROTTLE(10.0, "PPS同步偏移较大: %.6fs", current_rtk_data_.pps_offset);
        }
    }
    
    rtk_pub_.publish(current_rtk_data_);
    
    ROS_INFO_THROTTLE(2.0,
        "RTK 状态: 定位=%d, 卫星=%d, HDOP=%.1f, 位置=(%.6f, %.6f, %.1fm), 速度=%.1fkm/h",
        fix_status_, satellite_count_, hdop_,
        current_rtk_data_.latitude, current_rtk_data_.longitude,
        current_rtk_data_.altitude, current_rtk_data_.speed);
}

void RTKGPSParser::publishNavSatFix() {
    sensor_msgs::NavSatFix navsat_msg;
    navsat_msg.header.stamp = ros::Time::now();
    navsat_msg.header.frame_id = "rtk_gps";
    
    navsat_msg.latitude = current_rtk_data_.latitude;
    navsat_msg.longitude = current_rtk_data_.longitude;
    navsat_msg.altitude = current_rtk_data_.altitude;
    
    // 设置协方差
    navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    float h_var = current_rtk_data_.h_accuracy * current_rtk_data_.h_accuracy;
    float v_var = current_rtk_data_.v_accuracy * current_rtk_data_.v_accuracy;
    
    navsat_msg.position_covariance[0] = h_var;  // XX
    navsat_msg.position_covariance[4] = h_var;  // YY
    navsat_msg.position_covariance[8] = v_var;  // ZZ
    
    // 设置状态
    switch (fix_status_) {
        case 0: navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
        case 1: navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX; break;
        case 2: navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX; break;
        case 4: 
        case 5: navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; break;
        default: navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX; break;
    }
    
    navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    
    navsat_pub_.publish(navsat_msg);
}

void RTKGPSParser::publishTimeReference() {
    sensor_msgs::TimeReference time_ref_msg;
    time_ref_msg.header.stamp = ros::Time::now();
    time_ref_msg.header.frame_id = "rtk_gps";
    
    if (!current_rtk_data_.gps_time.isZero()) {
        time_ref_msg.time_ref = current_rtk_data_.gps_time;
        time_ref_msg.source = "RTK_GPS";
    }
    
    time_ref_pub_.publish(time_ref_msg);
}

void RTKGPSParser::featureEventCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        recordRTKEvent("feature_rich");
    }
}

void RTKGPSParser::recordRTKEvent(const std::string& event_type) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (fix_status_ < 2) {
        ROS_WARN("无法记录RTK事件: 定位质量不足 (%d)", fix_status_);
        return;
    }
    
    pending_event_.header.stamp = ros::Time::now();
    pending_event_.header.frame_id = "rtk_gps";
    pending_event_.event_type = event_type;
    pending_event_.latitude = current_rtk_data_.latitude;
    pending_event_.longitude = current_rtk_data_.longitude;
    pending_event_.altitude = current_rtk_data_.altitude;
    pending_event_.heading = current_rtk_data_.heading;
    pending_event_.position_accuracy = current_rtk_data_.h_accuracy;
    
    event_pending_ = true;
    
    ROS_INFO("RTK 事件等待中: %s 于 (%.6f, %.6f), 精度: %.3fm",
            event_type.c_str(),
            pending_event_.latitude,
            pending_event_.longitude,
            pending_event_.position_accuracy);
}

// 辅助函数实现
std::vector<std::string> RTKGPSParser::splitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    
    return tokens;
}

bool RTKGPSParser::validateChecksum(const std::string& sentence) {
    // NMEA校验和格式: $...*HH<CR><LF>
    size_t star_pos = sentence.find('*');
    if (star_pos == std::string::npos || star_pos < 1) {
        return false;
    }
    
    // 计算校验和
    uint8_t calculated = 0;
    for (size_t i = 1; i < star_pos; i++) {
        calculated ^= sentence[i];
    }
    
    // 解析校验和
    uint8_t received;
    std::stringstream ss;
    ss << std::hex << sentence.substr(star_pos + 1, 2);
    ss >> received;
    
    return calculated == received;
}

double RTKGPSParser::parseCoordinate(const std::string& coord, const std::string& direction) {
    if (coord.empty() || direction.empty()) {
        return 0.0;
    }
    
    try {
        // 格式: DDDMM.MMMMM
        size_t dot_pos = coord.find('.');
        if (dot_pos == std::string::npos || dot_pos < 2) {
            return 0.0;
        }
        
        int degrees = std::stoi(coord.substr(0, dot_pos - 2));
        double minutes = std::stod(coord.substr(dot_pos - 2));
        
        double decimal_degrees = degrees + minutes / 60.0;
        
        // 处理方向
        if (direction == "S" || direction == "W") {
            decimal_degrees = -decimal_degrees;
        }
        
        return decimal_degrees;
    } catch (const std::exception& e) {
        ROS_DEBUG("坐标解析错误: %s, %s", coord.c_str(), e.what());
        return 0.0;
    }
}

ros::Time RTKGPSParser::parseGPSTime(const std::string& time_str, const std::string& date_str) {
    if (time_str.empty()) {
        return ros::Time();
    }
    
    try {
        // 时间格式: HHMMSS.SSS
        int hour = std::stoi(time_str.substr(0, 2));
        int minute = std::stoi(time_str.substr(2, 2));
        int second = std::stoi(time_str.substr(4, 2));
        
        double fractional_seconds = 0.0;
        if (time_str.length() > 6) {
            fractional_seconds = std::stod("0." + time_str.substr(7));
        }
        
        // 处理日期
        int day = 1, month = 1, year = 2000;
        if (!date_str.empty() && date_str.length() == 6) {
            day = std::stoi(date_str.substr(0, 2));
            month = std::stoi(date_str.substr(2, 2));
            year = 2000 + std::stoi(date_str.substr(4, 2));
        }
        
        // 构建时间结构
        struct tm timeinfo = {};
        timeinfo.tm_year = year - 1900;
        timeinfo.tm_mon = month - 1;
        timeinfo.tm_mday = day;
        timeinfo.tm_hour = hour;
        timeinfo.tm_min = minute;
        timeinfo.tm_sec = second;
        
        time_t epoch_time = timegm(&timeinfo);
        
        if (epoch_time == -1) {
            ROS_WARN_THROTTLE(5.0, "GPS时间转换失败: %s %s", date_str.c_str(), time_str.c_str());
            return ros::Time();
        }
        
        ros::Time ros_time(epoch_time, static_cast<uint32_t>(fractional_seconds * 1e9));
        last_gps_time_ = ros_time;
        
        return ros_time;
    } catch (const std::exception& e) {
        ROS_DEBUG("GPS时间解析错误: %s", e.what());
        return ros::Time();
    }
}

void RTKGPSParser::run() {
    ros::Rate rate(10);
    
    // PPS监控线程（如果启用）
    std::thread pps_thread;
    if (enable_pps_) {
        pps_thread = std::thread([this]() {
            std::ifstream pps_file(pps_device_);
            if (!pps_file.is_open()) {
                ROS_WARN("无法打开PPS设备: %s", pps_device_.c_str());
                return;
            }
            
            while (running_ && ros::ok()) {
                char buffer[256];
                if (pps_file.getline(buffer, sizeof(buffer))) {
                    ros::Time pps_time = ros::Time::now();
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    pps_queue_.push(pps_time);
                    pps_synchronized_ = true;
                    
                    std_msgs::Bool pps_msg;
                    pps_msg.data = true;
                    pps_pub_.publish(pps_msg);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }
    
    while (ros::ok() && running_) {
        ros::spinOnce();
        
        static ros::Time last_status_check = ros::Time::now();
        if ((ros::Time::now() - last_status_check).toSec() > 5.0) {
            if (fix_status_ == 0) {
                ROS_WARN("RTK-GPS: 无定位");
            } else if (fix_status_ < 4) {
                ROS_WARN("RTK-GPS: 浮点解或单点解 (精度可能降低)");
            } else {
                ROS_INFO("RTK-GPS: 固定解 (高精度)");
            }
            last_status_check = ros::Time::now();
        }
        
        // 检查数据时效性
        static ros::Time last_valid_data = ros::Time::now();
        if ((ros::Time::now() - last_gps_time_).toSec() > 2.0 && !last_gps_time_.isZero()) {
            ROS_WARN_THROTTLE(5.0, "RTK数据超时: %.1f秒无更新", 
                            (ros::Time::now() - last_gps_time_).toSec());
        } else {
            last_valid_data = ros::Time::now();
        }
        
        rate.sleep();
    }
    
    if (enable_pps_ && pps_thread.joinable()) {
        pps_thread.join();
    }
}
