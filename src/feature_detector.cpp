#include "radar_obstacle_avoidance/feature_detector.h"
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <pcl_conversions/pcl_conversions.h>

FeatureDetector::FeatureDetector() : 
    current_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
    moving_average_entropy_(0.0),
    feature_rich_region_detected_(false)
{
    nh_.param("voxel_size", voxel_size_, 0.1f);
    nh_.param("k_search", k_search_, 30);
    nh_.param("entropy_threshold", entropy_threshold_, 1.5f);
    nh_.param("curvature_threshold", curvature_threshold_, 0.05f);
    nh_.param("planarity_threshold", planarity_threshold_, 0.5f);
    nh_.param("enable_debug", enable_debug_, true);
    
    // 订阅 FAST-LIVO2 处理后的点云
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/fast_livo/cloud_registered", 5, &FeatureDetector::cloudCallback, this);
    
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/fast_livo/odometry", 10, &FeatureDetector::odomCallback, this);
    
    feature_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/feature_points", 1);
    entropy_pub_ = nh_.advertise<std_msgs::Float32>("/feature_entropy", 1);
    event_pub_ = nh_.advertise<std_msgs::Bool>("/feature_rich_event", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/feature_markers", 1);
    
    ROS_INFO("特征检测器初始化完成");
}

void FeatureDetector::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*filtered_cloud);
    
    if (filtered_cloud->size() < k_search_ * 2) {
        ROS_WARN_THROTTLE(5.0, "点云太稀疏，无法进行特征检测");
        return;
    }
    
    FeatureMetrics metrics = computeFeatureMetrics(filtered_cloud);
    
    geometry_msgs::Pose current_pose;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_pose = current_odom_.pose.pose;
    }
    
    detectFeatureRichRegion(metrics, current_pose);
    
    if (enable_debug_) {
        publishVisualization(filtered_cloud, metrics);
    }
    
    std_msgs::Float32 entropy_msg;
    entropy_msg.data = metrics.entropy;
    entropy_pub_.publish(entropy_msg);
}

void FeatureDetector::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = *msg;
}

FeatureMetrics FeatureDetector::computeFeatureMetrics(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    FeatureMetrics metrics;
    
    metrics.entropy = computeEntropy(cloud);
    metrics.planarity = computePlanarity(cloud);
    metrics.linearity = computeLinearity(cloud);
    metrics.curvature = computeCurvature(cloud);
    metrics.feature_count = cloud->size();
    metrics.density = cloud->size() / 400.0;
    
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    
    for (const auto& point : cloud->points) {
        mean += Eigen::Vector3f(point.x, point.y, point.z);
    }
    mean /= cloud->size();
    
    for (const auto& point : cloud->points) {
        Eigen::Vector3f diff = Eigen::Vector3f(point.x, point.y, point.z) - mean;
        covariance += diff * diff.transpose();
    }
    covariance /= cloud->size();
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    metrics.eigenvalues.resize(3);
    metrics.eigenvalues[0] = solver.eigenvalues()[0];
    metrics.eigenvalues[1] = solver.eigenvalues()[1];
    metrics.eigenvalues[2] = solver.eigenvalues()[2];
    
    return metrics;
}

float FeatureDetector::computeEntropy(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (cloud->empty()) return 0.0f;
    
    const float grid_size = 0.5f;
    const int grid_dim = 40;
    std::vector<std::vector<int>> grid(grid_dim, std::vector<int>(grid_dim, 0));
    
    int total_points = 0;
    for (const auto& point : cloud->points) {
        int x_idx = static_cast<int>((point.x + 10.0) / grid_size);
        int y_idx = static_cast<int>((point.y + 10.0) / grid_size);
        
        if (x_idx >= 0 && x_idx < grid_dim && y_idx >= 0 && y_idx < grid_dim) {
            grid[x_idx][y_idx]++;
            total_points++;
        }
    }
    
    if (total_points == 0) return 0.0f;
    
    float entropy = 0.0f;
    for (int i = 0; i < grid_dim; ++i) {
        for (int j = 0; j < grid_dim; ++j) {
            if (grid[i][j] > 0) {
                float p = static_cast<float>(grid[i][j]) / total_points;
                entropy -= p * log2f(p);
            }
        }
    }
    
    return entropy;
}

float FeatureDetector::computePlanarity(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (cloud->size() < 10) return 0.0f;
    
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);
    
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    float lambda1 = eigenvalues[0];
    float lambda2 = eigenvalues[1];
    float lambda3 = eigenvalues[2];
    
    float planarity = (lambda2 - lambda1) / lambda3;
    return std::max(0.0f, std::min(1.0f, planarity));
}

float FeatureDetector::computeLinearity(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (cloud->size() < 10) return 0.0f;
    
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);
    
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    float lambda1 = eigenvalues[0];
    float lambda2 = eigenvalues[1];
    float lambda3 = eigenvalues[2];
    
    float linearity = (lambda3 - lambda2) / lambda3;
    return std::max(0.0f, std::min(1.0f, linearity));
}

float FeatureDetector::computeCurvature(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (cloud->size() < k_search_) return 0.0f;
    
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    
    float total_curvature = 0.0f;
    int valid_points = 0;
    
    for (size_t i = 0; i < cloud->size(); ++i) {
        std::vector<int> point_indices(k_search_);
        std::vector<float> point_distances(k_search_);
        
        if (kdtree.nearestKSearch(cloud->points[i], k_search_, point_indices, point_distances) > 0) {
            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
            for (int idx : point_indices) {
                centroid += Eigen::Vector3f(cloud->points[idx].x,
                                           cloud->points[idx].y,
                                           cloud->points[idx].z);
            }
            centroid /= point_indices.size();
            
            Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
            for (int idx : point_indices) {
                Eigen::Vector3f diff = Eigen::Vector3f(cloud->points[idx].x,
                                                      cloud->points[idx].y,
                                                      cloud->points[idx].z) - centroid;
                covariance += diff * diff.transpose();
            }
            covariance /= point_indices.size();
            
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
            Eigen::Vector3f eigenvalues = solver.eigenvalues();
            
            float curvature = eigenvalues[0] / (eigenvalues.sum() + 1e-6);
            total_curvature += curvature;
            valid_points++;
        }
    }
    
    return (valid_points > 0) ? total_curvature / valid_points : 0.0f;
}

void FeatureDetector::detectFeatureRichRegion(const FeatureMetrics& metrics, const geometry_msgs::Pose& pose) {
    entropy_history_.push(metrics.entropy);
    if (entropy_history_.size() > history_size_) {
        entropy_history_.pop();
    }
    
    float sum = 0.0f;
    std::queue<float> temp_queue = entropy_history_;
    while (!temp_queue.empty()) {
        sum += temp_queue.front();
        temp_queue.pop();
    }
    moving_average_entropy_ = sum / entropy_history_.size();
    
    bool is_feature_rich = (metrics.entropy > entropy_threshold_) &&
                          (metrics.curvature > 0.01 && metrics.curvature < 0.1) &&
                          (metrics.planarity > planarity_threshold_);
    
    static bool last_state = false;
    if (is_feature_rich && !last_state) {
        ROS_WARN("====== 检测到特征丰富区域 ======");
        ROS_INFO("信息熵: %.3f (阈值: %.3f)", metrics.entropy, entropy_threshold_);
        ROS_INFO("曲率: %.4f", metrics.curvature);
        ROS_INFO("平面性: %.3f", metrics.planarity);
        ROS_INFO("位置: (%.2f, %.2f, %.2f)", pose.position.x, pose.position.y, pose.position.z);
        ROS_INFO("特征点数量: %d", metrics.feature_count);
        
        std_msgs::Bool event_msg;
        event_msg.data = true;
        event_pub_.publish(event_msg);
        
        feature_rich_region_detected_ = true;
    } else if (!is_feature_rich && last_state) {
        feature_rich_region_detected_ = false;
    }
    
    last_state = is_feature_rich;
}

void FeatureDetector::publishVisualization(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                          const FeatureMetrics& metrics) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    for (size_t i = 0; i < cloud->size(); ++i) {
        pcl::PointXYZRGB point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        
        float normalized_entropy = std::min(1.0f, metrics.entropy / 3.0f);
        point.r = static_cast<uint8_t>(normalized_entropy * 255);
        point.g = 100;
        point.b = static_cast<uint8_t>((1.0 - normalized_entropy) * 255);
        
        colored_cloud->push_back(point);
    }
    
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*colored_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    feature_pub_.publish(cloud_msg);
    
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "feature_info";
    text_marker.id = 0;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position.x = 0;
    text_marker.pose.position.y = 0;
    text_marker.pose.position.z = 2;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;
    
    char text[256];
    snprintf(text, sizeof(text),
             "信息熵: %.2f\n平面性: %.2f\n曲率: %.3f\n点数: %d",
             metrics.entropy, metrics.planarity, 
             metrics.curvature, metrics.feature_count);
    text_marker.text = text;
    
    marker_pub_.publish(text_marker);
}

void FeatureDetector::run() {
    ros::Rate rate(10);
    
    while (ros::ok()) {
        ros::spinOnce();
        
        static ros::Time last_print = ros::Time::now();
        if ((ros::Time::now() - last_print).toSec() > 2.0) {
            ROS_INFO_THROTTLE(2.0, 
                "特征检测 - 移动平均信息熵: %.3f, 状态: %s",
                moving_average_entropy_,
                feature_rich_region_detected_ ? "丰富" : "稀疏");
            last_print = ros::Time::now();
        }
        
        rate.sleep();
    }
}
