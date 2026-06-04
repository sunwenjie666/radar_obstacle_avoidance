#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudRelay {
public:
    PointCloudRelay() {
        // 订阅FAST-LIVO2的点云话题
        sub_ = nh_.subscribe("/cloud_registered", 1, &PointCloudRelay::callback, this);
        // 发布到FIESTA期望的话题
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud", 1);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // 直接转发，不修改任何内容
        pub_.publish(cloud_msg);
        ROS_DEBUG("PointCloud relayed.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_relay");
    PointCloudRelay relay;
    ros::spin();
    return 0;
}
