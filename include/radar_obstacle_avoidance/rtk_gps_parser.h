#ifndef RTK_GPS_PARSER_H
#define RTK_GPS_PARSER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include "radar_obstacle_avoidance/RTKData.h"
#include "radar_obstacle_avoidance/RTKEvent.h"
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>

using namespace radar_obstacle_avoidance;
class RTKGPSParser {
private:
    ros::NodeHandle nh_;
    ros::Publisher rtk_pub_;
    ros::Publisher navsat_pub_;
    ros::Publisher event_pub_;
    ros::Publisher time_ref_pub_;
    ros::Publisher pps_pub_;
    ros::Subscriber feature_event_sub_;
    
    std::unique_ptr<serial::Serial> serial_port_;
    std::thread serial_thread_;
    std::mutex data_mutex_;
    std::atomic<bool> running_;
    
    RTKData current_rtk_data_;
    std::queue<std::string> nmea_queue_;
    std::queue<ros::Time> pps_queue_;
    
    std::string port_;
    int baud_rate_;
    bool enable_pps_;
    std::string pps_device_;
    
    bool pps_synchronized_;
    ros::Time last_pps_time_;
    ros::Time last_gps_time_;
    int fix_status_;
    float hdop_;
    int satellite_count_;
    
    bool event_pending_;
    RTKEvent pending_event_;
    
public:
    RTKGPSParser();
    ~RTKGPSParser();
    
    bool initSerial();
    void serialReadThread();
    void processNMEASentence(const std::string& sentence);
    
    bool parseGGA(const std::vector<std::string>& fields);
    bool parseRMC(const std::vector<std::string>& fields);
    bool parseGSA(const std::vector<std::string>& fields);
    bool parseVTG(const std::vector<std::string>& fields);
    bool parseZDA(const std::vector<std::string>& fields);
    bool parseGST(const std::vector<std::string>& fields);
    
    void publishRTKData();
    void publishNavSatFix();
    void publishTimeReference();
    
    void featureEventCallback(const std_msgs::Bool::ConstPtr& msg);
    void recordRTKEvent(const std::string& event_type);
    
    void run();
    
private:
    std::vector<std::string> splitString(const std::string& str, char delimiter);
    bool validateChecksum(const std::string& sentence);
    double parseCoordinate(const std::string& coord, const std::string& direction);
    ros::Time parseGPSTime(const std::string& time_str, const std::string& date_str);
};

#endif
