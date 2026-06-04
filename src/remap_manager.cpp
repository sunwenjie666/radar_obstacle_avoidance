#include "radar_obstacle_avoidance/remap_manager.h"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <chrono>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

using namespace radar_obstacle_avoidance;
RemapManager::RemapManager() :
    remap_in_progress_(false),
    remap_count_(0),
    global_map_(new pcl::PointCloud<pcl::PointXYZI>()),
    path_clearance_(1.0)
{
    nh_.param("enable_auto_remap", enable_auto_remap_, true);
    nh_.param("feature_entropy_threshold", feature_entropy_threshold_, 1.5f);
    nh_.param("residual_threshold", residual_threshold_, 0.1f);
    nh_.param("min_remap_interval", min_remap_interval_, 10.0f);
    nh_.param("map_resolution", map_resolution_, 0.1f);
    
    int map_type_int;
    nh_.param("map_type", map_type_int, 0);
    map_type_ = static_cast<MapType>(map_type_int);
    
    int planner_type_int;
    nh_.param("planner_type", planner_type_int, 0);
    planner_type_ = static_cast<PlanningAlgorithm>(planner_type_int);
    
    feature_event_sub_ = nh_.subscribe<std_msgs::Bool>(
        "/feature_rich_event", 1, &RemapManager::featureEventCallback, this);
    
    rtk_event_sub_ = nh_.subscribe<RTKEvent>(
        "/rtk/event", 10, &RemapManager::rtkEventCallback, this);
    
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/fast_livo/odometry", 10, &RemapManager::odomCallback, this);
    
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/fast_livo/cloud_registered", 5, &RemapManager::cloudCallback, this);
    
    residual_sub_ = nh_.subscribe<std_msgs::Float32>(
        "/fast_livo/residuals", 10, &RemapManager::residualCallback, this);
    
    remap_cmd_pub_ = nh_.advertise<RemapCommand>("/remap/command", 1, true);
    path_pub_ = nh_.advertise<PathPlanning>("/path/planning", 1);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/remap/markers", 1);
    status_pub_ = nh_.advertise<std_msgs::Float32>("/remap/status", 1);
    
    if (map_type_ == MAP_OCTOMAP) {
        octree_.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(map_resolution_));
        octree_->setInputCloud(global_map_);
    }
    
    ROS_INFO("重新建图管理器初始化完成");
}

void RemapManager::featureEventCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && enable_auto_remap_) {
        ROS_WARN("检测到特征丰富区域，考虑重新建图...");
    }
}

void RemapManager::rtkEventCallback(const RTKEvent::ConstPtr& msg) {
    if (msg->event_triggered) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        event_queue_.push(*msg);
        ROS_INFO("RTK事件已加入队列: %s 于 (%.6f, %.6f)", 
                 msg->event_type.c_str(),
                 msg->latitude, msg->longitude);
    }
}

void RemapManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    static geometry_msgs::PoseStamped current_pose;
    current_pose.header = msg->header;
    current_pose.pose = msg->pose.pose;
    
    static ros::Time last_check = ros::Time::now();
    if ((ros::Time::now() - last_check).toSec() > 1.0) {
        if (!event_queue_.empty() && !remap_in_progress_) {
            RTKEvent event;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                event = event_queue_.front();
            }
            
            if (shouldRemap(event, *msg, 0.05f)) {
                triggerRemap(event, "feature_rich");
                event_queue_.pop();
            }
        }
        last_check = ros::Time::now();
    }
}

void RemapManager::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    
    updateGlobalMap(cloud);
    publishMapVisualization();
}

void RemapManager::residualCallback(const std_msgs::Float32::ConstPtr& msg) {
    if (msg->data > residual_threshold_ && enable_auto_remap_) {
        ROS_WARN("检测到高残差 (%.4f)，可能需要重新建图", msg->data);
        
        if (!event_queue_.empty()) {
            RTKEvent event;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                event = event_queue_.front();
            }
            
            triggerRemap(event, "high_residual");
        }
    }
}

bool RemapManager::shouldRemap(const RTKEvent& event, 
                               const nav_msgs::Odometry& odom,
                               float current_residual) {
    if ((ros::Time::now() - last_remap_time_).toSec() < min_remap_interval_) {
        ROS_DEBUG("重新建图过于频繁，跳过");
        return false;
    }
    
    if (event.position_accuracy > 0.1) {
        ROS_WARN("定位精度太低 (%.3fm)，不适合重新建图", event.position_accuracy);
        return false;
    }
    
    if (event.event_type != "feature_rich") {
        ROS_DEBUG("事件类型不适合重新建图: %s", event.event_type.c_str());
        return false;
    }
    
    if (current_residual > residual_threshold_) {
        ROS_WARN("高残差 (%.4f)，适合重新建图", current_residual);
        return true;
    }
    
    return true;
}

void RemapManager::triggerRemap(const RTKEvent& event, const std::string& reason) {
    if (remap_in_progress_) {
        ROS_WARN("重新建图已在进程中，跳过");
        return;
    }
    
    ROS_WARN("========== 触发重新建图 ==========");
    ROS_INFO("原因: %s", reason.c_str());
    ROS_INFO("位置: (%.6f, %.6f, %.2f)", event.latitude, event.longitude, event.altitude);
    ROS_INFO("航向: %.1f°, 精度: %.3fm", event.heading, event.position_accuracy);
    
    remap_in_progress_ = true;
    last_remap_time_ = ros::Time::now();
    remap_count_++;
    
    executeRemap(event);
    
    RemapCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "map";
    cmd.remap_trigger = true;
    cmd.trigger_reason = reason;
    cmd.origin_latitude = event.latitude;
    cmd.origin_longitude = event.longitude;
    cmd.origin_altitude = event.altitude;
    cmd.origin_heading = event.heading;
    cmd.position_accuracy = event.position_accuracy;
    cmd.map_resolution = map_resolution_;
    cmd.map_frame_id = "map_remap_" + std::to_string(remap_count_);
    
    remap_cmd_pub_.publish(cmd);
    
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = event.latitude;
    goal.pose.position.y = event.longitude;
    goal.pose.position.z = event.altitude + 2.0;
    
    geometry_msgs::PoseStamped start;
    start.header.frame_id = "map";
    start.pose.orientation.w = 1.0;
    
    if (planPath(start, goal)) {
        ROS_INFO("路径规划成功，%zu 个航点", current_path_.size());
    }
    
    remap_in_progress_ = false;
}

void RemapManager::executeRemap(const RTKEvent& event) {
    ROS_INFO("执行重新建图程序...");
    
    std::string map_filename = "/tmp/map_before_remap_" + std::to_string(remap_count_) + ".pcd";
    saveMapToFile(map_filename);
    
    geometry_msgs::Pose center;
    center.position.x = event.latitude;
    center.position.y = event.longitude;
    center.position.z = event.altitude;
    
    clearOldMapRegion(center, 20.0);
    
    map_origin_lat_ = event.latitude;
    map_origin_lon_ = event.longitude;
    map_origin_alt_ = event.altitude;
    
    if (map_type_ == MAP_OCTOMAP && !global_map_->empty()) {
        octree_->deleteTree();
        octree_->setInputCloud(global_map_);
        octree_->addPointsFromInputCloud();
    }
    
    ROS_INFO("重新建图完成。新原点: (%.6f, %.6f, %.2f)",
             map_origin_lat_, map_origin_lon_, map_origin_alt_);
}

void RemapManager::updateGlobalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (cloud->empty()) return;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setLeafSize(map_resolution_, map_resolution_, map_resolution_);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*filtered_cloud);
    
    *global_map_ += *filtered_cloud;
    
    if (global_map_->size() > 1000000) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::RandomSample<pcl::PointXYZI> random_sample;
        random_sample.setInputCloud(global_map_);
        random_sample.setSample(500000);
        random_sample.filter(*downsampled);
        
        global_map_->swap(*downsampled);
    }
    
    if (map_type_ == MAP_OCTOMAP && octree_) {
        octree_->deleteTree();
        octree_->setInputCloud(global_map_);
        octree_->addPointsFromInputCloud();
    }
    
    ROS_DEBUG_THROTTLE(5.0, "全局地图更新: %zu 点", global_map_->size());
}

bool RemapManager::planPath(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            float max_search_time) {
    if (global_map_->empty()) {
        ROS_WARN("无法规划路径: 地图为空");
        return true;
    }
    
    bool success = false;
    auto start_time = std::chrono::steady_clock::now();
    
    switch (planner_type_) {
        case PLANNER_ASTAR:
            success = planPathAStar(start, goal);
            break;
        default:
            ROS_WARN("未知的规划器类型: %d", planner_type_);
            return false;
    }
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (success) {
        PathPlanning path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        path_msg.waypoints = current_path_;
        path_msg.feasible = true;
        
        float total_length = 0.0;
        for (size_t i = 1; i < current_path_.size(); ++i) {
            float dx = current_path_[i].pose.position.x - current_path_[i-1].pose.position.x;
            float dy = current_path_[i].pose.position.y - current_path_[i-1].pose.position.y;
            float dz = current_path_[i].pose.position.z - current_path_[i-1].pose.position.z;
            float segment_length = sqrt(dx*dx + dy*dy + dz*dz);
            path_msg.segment_lengths.push_back(segment_length);
            total_length += segment_length;
        }
        path_msg.total_length = total_length;
        
        path_clearance_ = std::numeric_limits<float>::max();
        for (const auto& waypoint : current_path_) {
            float clearance = computeClearance(waypoint.pose);
            if (clearance < path_clearance_) {
                path_clearance_ = clearance;
            }
        }
        path_msg.clearance = path_clearance_;
        
        path_pub_.publish(path_msg);
        publishPathVisualization();
        
        ROS_INFO("路径规划成功，耗时 %ld ms", duration.count());
        ROS_INFO("总长度: %.2f m", total_length);
        ROS_INFO("最小间隙: %.2f m", path_clearance_);
        ROS_INFO("航点数: %zu", current_path_.size());
    } else {
        ROS_WARN("路径规划失败，耗时 %ld ms", duration.count());
    }
    
    return success;
}

bool RemapManager::planPathAStar(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal) {
    ROS_INFO("使用A*规划路径从 (%.2f, %.2f, %.2f) 到 (%.2f, %.2f, %.2f)",
             start.pose.position.x, start.pose.position.y, start.pose.position.z,
             goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    
    current_path_.clear();
    current_path_.push_back(start);
    
    int num_points = 10;
    for (int i = 1; i < num_points; ++i) {
        float t = static_cast<float>(i) / num_points;
        geometry_msgs::PoseStamped waypoint;
        waypoint.header = start.header;
        
        waypoint.pose.position.x = start.pose.position.x + 
                                   t * (goal.pose.position.x - start.pose.position.x);
        waypoint.pose.position.y = start.pose.position.y + 
                                   t * (goal.pose.position.y - start.pose.position.y);
        waypoint.pose.position.z = start.pose.position.z + 
                                   t * (goal.pose.position.z - start.pose.position.z);
        
        if (checkCollision(waypoint.pose, 0.5)) {
            ROS_WARN("航点 %d 有碰撞，调整中...", i);
            waypoint.pose.position.z += 1.0;
        }
        
        current_path_.push_back(waypoint);
    }
    
    current_path_.push_back(goal);
    
    return true;
}

bool RemapManager::checkCollision(const geometry_msgs::Pose& pose, float radius) {
    if (global_map_->empty() || !octree_) {
        return false;
    }
    
    pcl::PointXYZI search_point;
    search_point.x = pose.position.x;
    search_point.y = pose.position.y;
    search_point.z = pose.position.z;
    
    std::vector<int> point_indices;
    std::vector<float> point_distances;
    
    if (octree_->radiusSearch(search_point, radius, point_indices, point_distances) > 0) {
        return true;
    }
    
    return false;
}

float RemapManager::computeClearance(const geometry_msgs::Pose& pose) {
    if (global_map_->empty() || !octree_) {
        return std::numeric_limits<float>::max();
    }
    
    pcl::PointXYZI search_point;
    search_point.x = pose.position.x;
    search_point.y = pose.position.y;
    search_point.z = pose.position.z;
    
    std::vector<int> point_indices(1);
    std::vector<float> point_distances(1);
    
    if (octree_->nearestKSearch(search_point, 1, point_indices, point_distances) > 0) {
        return sqrt(point_distances[0]);
    }
    
    return std::numeric_limits<float>::max();
}

void RemapManager::publishMapVisualization() {
    if (global_map_->empty()) return;
    
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*global_map_, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "map";
    
    map_pub_.publish(map_msg);
    
    visualization_msgs::MarkerArray marker_array;
    
    visualization_msgs::Marker origin_marker;
    origin_marker.header.frame_id = "map";
    origin_marker.header.stamp = ros::Time::now();
    origin_marker.ns = "map_origin";
    origin_marker.id = 0;
    origin_marker.type = visualization_msgs::Marker::SPHERE;
    origin_marker.action = visualization_msgs::Marker::ADD;
    origin_marker.pose.position.x = 0;
    origin_marker.pose.position.y = 0;
    origin_marker.pose.position.z = 0;
    origin_marker.scale.x = 0.5;
    origin_marker.scale.y = 0.5;
    origin_marker.scale.z = 0.5;
    origin_marker.color.r = 1.0;
    origin_marker.color.g = 0.0;
    origin_marker.color.b = 0.0;
    origin_marker.color.a = 1.0;
    origin_marker.lifetime = ros::Duration();
    
    marker_array.markers.push_back(origin_marker);
    marker_pub_.publish(marker_array);
}

void RemapManager::saveMapToFile(const std::string& filename) {
    // TODO: 实现保存地图到文件
    ROS_INFO("Saving map to %s (not implemented)", filename.c_str());
}

void RemapManager::clearOldMapRegion(const geometry_msgs::Pose& center, float radius) {
    // TODO: 实现清除旧地图区域
    ROS_INFO("Clearing old map region (not implemented)");
}

void RemapManager::publishPathVisualization() {
    // TODO: 实现路径可视化
    ROS_DEBUG("Publishing path visualization (not implemented)");
}

void RemapManager::publishRemapStatus() {
    // TODO: 实现发布重新建图状态
    ROS_DEBUG("Publishing remap status (not implemented)");
}

void RemapManager::run() {
    ros::Rate rate(10);
    
    while (ros::ok()) {
        ros::spinOnce();
        
        static ros::Time last_status_pub = ros::Time::now();
        if ((ros::Time::now() - last_status_pub).toSec() > 1.0) {
            publishRemapStatus();
            last_status_pub = ros::Time::now();
        }
        
        static ros::Time last_map_save = ros::Time::now();
        if ((ros::Time::now() - last_map_save).toSec() > 30.0) {
            std::string auto_save_file = "/tmp/auto_save_map_" + 
                                        std::to_string(ros::Time::now().toSec()) + ".pcd";
            saveMapToFile(auto_save_file);
            last_map_save = ros::Time::now();
        }
        
        rate.sleep();
    }
}
