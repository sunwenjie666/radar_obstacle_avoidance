#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/pca.h>
#include <Eigen/Core>
#include <queue>
#include <mutex>
#include <atomic>

struct FeatureMetrics {
    float entropy;
    float planarity;
    float linearity;
    float curvature;
    int feature_count;
    float density;
    std::vector<float> eigenvalues;
};

class FeatureDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher feature_pub_;
    ros::Publisher entropy_pub_;
    ros::Publisher event_pub_;
    ros::Publisher marker_pub_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;
    nav_msgs::Odometry current_odom_;
    std::mutex cloud_mutex_;
    std::mutex odom_mutex_;
    
    float voxel_size_;
    int k_search_;
    float entropy_threshold_;
    float curvature_threshold_;
    float planarity_threshold_;
    bool enable_debug_;
    
    std::queue<float> entropy_history_;
    const size_t history_size_ = 100;
    float moving_average_entropy_;
    std::atomic<bool> feature_rich_region_detected_;
    
public:
    FeatureDetector();
    
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    
    FeatureMetrics computeFeatureMetrics(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    float computeEntropy(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    float computePlanarity(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    float computeLinearity(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    float computeCurvature(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    void detectFeatureRichRegion(const FeatureMetrics& metrics, const geometry_msgs::Pose& pose);
    void publishVisualization(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const FeatureMetrics& metrics);
    void run();
};

#endif
