#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "radar_obstacle_avoidance/RTKData.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace radar_obstacle_avoidance;
class TimeSyncNode {
private:
    ros::NodeHandle nh_;
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<RTKData> rtk_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,
        nav_msgs::Odometry,
        RTKData> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    
    ros::Publisher synced_cloud_pub_;
    ros::Publisher synced_odom_pub_;
    ros::Publisher utm_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    std::string utm_zone_;
    bool northern_hemisphere_;
    bool use_utm_;
    
public:
    TimeSyncNode() : 
        cloud_sub_(nh_, "/fast_livo/cloud_registered", 10),
        odom_sub_(nh_, "/fast_livo/odometry", 10),
        rtk_sub_(nh_, "/rtk/data", 10)
    {
        nh_.param("use_utm", use_utm_, true);
        
        sync_.reset(new Sync(SyncPolicy(10), cloud_sub_, odom_sub_, rtk_sub_));
        sync_->registerCallback(boost::bind(&TimeSyncNode::syncCallback, this, _1, _2, _3));
        
        synced_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/synced/cloud", 10);
        synced_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/synced/odometry", 10);
        utm_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/utm/pose", 10);
        
        ROS_INFO("时间同步节点初始化完成");
    }
    
    void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                     const nav_msgs::OdometryConstPtr& odom_msg,
                     const RTKDataConstPtr& rtk_msg) {
        ros::Time cloud_time = cloud_msg->header.stamp;
        ros::Time odom_time = odom_msg->header.stamp;
        ros::Time rtk_time = rtk_msg->header.stamp;
        
        double max_diff = 0.1;
        if (fabs((cloud_time - rtk_time).toSec()) > max_diff ||
            fabs((odom_time - rtk_time).toSec()) > max_diff) {
            ROS_WARN_THROTTLE(1.0, "时间同步警告: 时间戳差异过大");
        }
        
        sensor_msgs::PointCloud2 synced_cloud = *cloud_msg;
        synced_cloud.header.stamp = ros::Time::now();
        synced_cloud_pub_.publish(synced_cloud);
        
        publishTFTransform(*rtk_msg, *odom_msg);
        
        ROS_DEBUG_THROTTLE(2.0, 
            "时间同步: 点云=%f, 里程计=%f, RTK=%f, 差异=%.3fs",
            cloud_time.toSec(), odom_time.toSec(), rtk_time.toSec(),
            (ros::Time::now() - rtk_time).toSec());
    }
    
    void publishTFTransform(const RTKData& rtk_data, const nav_msgs::Odometry& odom) {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "odom";
        transform_stamped.transform.translation.x = odom.pose.pose.position.x;
        transform_stamped.transform.translation.y = odom.pose.pose.position.y;
        transform_stamped.transform.translation.z = odom.pose.pose.position.z;
        transform_stamped.transform.rotation = odom.pose.pose.orientation;
        
        tf_broadcaster_.sendTransform(transform_stamped);
        
        geometry_msgs::TransformStamped transform2;
        transform2.header.stamp = ros::Time::now();
        transform2.header.frame_id = "odom";
        transform2.child_frame_id = "base_link";
        transform2.transform.rotation.w = 1.0;
        
        tf_broadcaster_.sendTransform(transform2);
    }
    
    void run() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "time_sync_node");
    
    TimeSyncNode node;
    node.run();
    
    return 0;
}
