#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <map>
#include <string>
#include <cstdlib>  // for system()

class SystemMonitor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber feature_entropy_sub_;
    ros::Subscriber rtk_fix_sub_;
    ros::Publisher diag_pub_;          // 诊断信息发布器

    double odom_freq_;
    double cloud_freq_;
    double entropy_;
    int fix_status_;

    ros::Time last_odom_time_;
    ros::Time last_cloud_time_;

    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        if (!last_odom_time_.isZero()) {
            double dt = (msg->header.stamp - last_odom_time_).toSec();
            if (dt > 0) odom_freq_ = 1.0 / dt;
        }
        last_odom_time_ = msg->header.stamp;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        if (!last_cloud_time_.isZero()) {
            double dt = (msg->header.stamp - last_cloud_time_).toSec();
            if (dt > 0) cloud_freq_ = 1.0 / dt;
        }
        last_cloud_time_ = msg->header.stamp;
    }

    void entropyCallback(const std_msgs::Float32ConstPtr& msg) {
        entropy_ = msg->data;
    }

    void rtkCallback(const std_msgs::Int32ConstPtr& msg) {
        fix_status_ = msg->data;
    }

    // ---------- 视觉监控辅助函数 ----------
    bool checkNodeAlive(const std::string& node_name) {
        std::string cmd = "rosnode list | grep " + node_name + " > /dev/null 2>&1";
        int ret = system(cmd.c_str());
        return (ret == 0);
    }

    double getTopicFrequency(const std::string& topic_name) {
        // 简单实现：使用 rostopic hz 一次（非阻塞，仅作占位）
        // 更精确的实现需要订阅话题统计，为简化先返回默认值
        // 实际项目中可改进
        std::string cmd = "rostopic hz " + topic_name + " -n 1 2>/dev/null | grep average | awk '{print $3}'";
        FILE* fp = popen(cmd.c_str(), "r");
        if (fp) {
            char buf[32];
            if (fgets(buf, sizeof(buf), fp) != nullptr) {
                double freq = atof(buf);
                pclose(fp);
                return freq;
            }
            pclose(fp);
        }
        return 0.0;
    }

    void checkVisionSystem() {
        bool yolo_alive = checkNodeAlive("yolo_detector");
        bool tracker_alive = checkNodeAlive("object_tracker");
        bool fusion_alive = checkNodeAlive("sensor_fusion");

        double yolo_freq = getTopicFrequency("/yolo/detections");
        double camera_freq = getTopicFrequency("/camera/image_raw");

        diagnostic_msgs::DiagnosticArray diag_array;
        diag_array.header.stamp = ros::Time::now();

        // YOLO检测器状态
        diagnostic_msgs::DiagnosticStatus yolo_status;
        yolo_status.name = "YOLO Detector";
        yolo_status.level = yolo_alive ? diagnostic_msgs::DiagnosticStatus::OK 
                                       : diagnostic_msgs::DiagnosticStatus::ERROR;
        yolo_status.message = yolo_alive ? "Running" : "Not responding";
        diag_array.status.push_back(yolo_status);

        // 话题频率（可选）
        diagnostic_msgs::DiagnosticStatus freq_status;
        freq_status.name = "Vision Topics";
        freq_status.level = diagnostic_msgs::DiagnosticStatus::OK;
        diagnostic_msgs::KeyValue kv_yolo_freq, kv_cam_freq;
        kv_yolo_freq.key = "/yolo/detections freq (Hz)";
        kv_yolo_freq.value = std::to_string(yolo_freq);
        kv_cam_freq.key = "/camera/image_raw freq (Hz)";
        kv_cam_freq.value = std::to_string(camera_freq);
        freq_status.values.push_back(kv_yolo_freq);
        freq_status.values.push_back(kv_cam_freq);
        diag_array.status.push_back(freq_status);

        diag_pub_.publish(diag_array);
    }

public:
    SystemMonitor() : nh_("~"),
        odom_freq_(0.0), cloud_freq_(0.0), entropy_(0.0), fix_status_(0) {
        odom_sub_ = nh_.subscribe("/fast_livo/odometry", 5, &SystemMonitor::odomCallback, this);
        cloud_sub_ = nh_.subscribe("/fast_livo/cloud_registered", 5, &SystemMonitor::cloudCallback, this);
        feature_entropy_sub_ = nh_.subscribe("/feature_entropy", 5, &SystemMonitor::entropyCallback, this);
        rtk_fix_sub_ = nh_.subscribe("/rtk/fix_status", 5, &SystemMonitor::rtkCallback, this);
        diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 5);
        ROS_INFO("SystemMonitor initialized");
    }

    void run() {
        ros::Rate rate(1); // 1 Hz
        while (ros::ok()) {
            ros::spinOnce();

            // 常规诊断（雷达、里程计、RTK等）
            diagnostic_msgs::DiagnosticArray diag;
            diag.header.stamp = ros::Time::now();

            // Odometry status
            diagnostic_msgs::DiagnosticStatus odom_status;
            odom_status.name = "Odometry";
            odom_status.hardware_id = "FAST-LIVO2";
            if (odom_freq_ > 9.0) {
                odom_status.level = diagnostic_msgs::DiagnosticStatus::OK;
                odom_status.message = "Running normally";
            } else {
                odom_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
                odom_status.message = "Low frequency or no data";
            }
            diagnostic_msgs::KeyValue kv_odom_freq;
            kv_odom_freq.key = "Frequency (Hz)";
            kv_odom_freq.value = std::to_string(odom_freq_);
            odom_status.values.push_back(kv_odom_freq);
            diag.status.push_back(odom_status);

            // Point cloud status
            diagnostic_msgs::DiagnosticStatus cloud_status;
            cloud_status.name = "PointCloud";
            cloud_status.hardware_id = "LiDAR";
            if (cloud_freq_ > 9.0) {
                cloud_status.level = diagnostic_msgs::DiagnosticStatus::OK;
                cloud_status.message = "Receiving data";
            } else {
                cloud_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
                cloud_status.message = "Low frequency or no data";
            }
            diagnostic_msgs::KeyValue kv_cloud_freq;
            kv_cloud_freq.key = "Frequency (Hz)";
            kv_cloud_freq.value = std::to_string(cloud_freq_);
            cloud_status.values.push_back(kv_cloud_freq);
            diag.status.push_back(cloud_status);

            // Feature entropy
            diagnostic_msgs::DiagnosticStatus feature_status;
            feature_status.name = "Feature Detection";
            feature_status.level = diagnostic_msgs::DiagnosticStatus::OK;
            feature_status.message = "Active";
            diagnostic_msgs::KeyValue kv_entropy;
            kv_entropy.key = "Entropy";
            kv_entropy.value = std::to_string(entropy_);
            feature_status.values.push_back(kv_entropy);
            diag.status.push_back(feature_status);

            // RTK status
            diagnostic_msgs::DiagnosticStatus rtk_status;
            rtk_status.name = "RTK-GPS";
            rtk_status.hardware_id = "GPS";
            if (fix_status_ >= 4) {
                rtk_status.level = diagnostic_msgs::DiagnosticStatus::OK;
                rtk_status.message = "RTK fixed";
            } else if (fix_status_ >= 2) {
                rtk_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
                rtk_status.message = "Float solution";
            } else {
                rtk_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
                rtk_status.message = "No fix";
            }
            diagnostic_msgs::KeyValue kv_fix;
            kv_fix.key = "Fix status";
            kv_fix.value = std::to_string(fix_status_);
            rtk_status.values.push_back(kv_fix);
            diag.status.push_back(rtk_status);

            diag_pub_.publish(diag);

            // 调用视觉监控
            checkVisionSystem();

            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "system_monitor_node");
    SystemMonitor monitor;
    monitor.run();
    return 0;
}
