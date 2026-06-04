#ifndef REMAP_MANAGER_H
#define REMAP_MANAGER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "radar_obstacle_avoidance/RTKEvent.h"
#include "radar_obstacle_avoidance/RemapCommand.h"
#include "radar_obstacle_avoidance/PathPlanning.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <queue>
#include <mutex>
#include <atomic>
#include <memory>

using namespace radar_obstacle_avoidance;
enum MapType {
    MAP_OCTOMAP,
    MAP_VOXEL,
    MAP_ELEVATION,
    MAP_POINTCLOUD
};

enum PlanningAlgorithm {
    PLANNER_ASTAR,
    PLANNER_RRT,
    PLANNER_PRM,
    PLANNER_DWA
};

class RemapManager {
private:
    ros::NodeHandle nh_;
    
    ros::Subscriber feature_event_sub_;
    ros::Subscriber rtk_event_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber residual_sub_;
    
    ros::Publisher remap_cmd_pub_;
    ros::Publisher path_pub_;
    ros::Publisher map_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher status_pub_;
    
    bool enable_auto_remap_;
    float feature_entropy_threshold_;
    float residual_threshold_;
    float min_remap_interval_;
    float map_resolution_;
    MapType map_type_;
    PlanningAlgorithm planner_type_;
    
    std::atomic<bool> remap_in_progress_;
    ros::Time last_remap_time_;
    int remap_count_;
    std::queue<RTKEvent> event_queue_;
    std::mutex queue_mutex_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
    std::shared_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>> octree_;
    float map_origin_lat_, map_origin_lon_, map_origin_alt_;
    
    std::vector<geometry_msgs::PoseStamped> current_path_;
    geometry_msgs::PoseStamped current_goal_;
    float path_clearance_;
    
public:
    RemapManager();
    
    void featureEventCallback(const std_msgs::Bool::ConstPtr& msg);
    void rtkEventCallback(const RTKEvent::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void residualCallback(const std_msgs::Float32::ConstPtr& msg);
    
    bool shouldRemap(const RTKEvent& event, const nav_msgs::Odometry& odom, float current_residual);
    void triggerRemap(const RTKEvent& event, const std::string& reason);
    void executeRemap(const RTKEvent& event);
    
    void updateGlobalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void clearOldMapRegion(const geometry_msgs::Pose& center, float radius);
    void saveMapToFile(const std::string& filename);
    void loadMapFromFile(const std::string& filename);
    
    bool planPath(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  float max_search_time = 5.0);
    bool planPathAStar(const geometry_msgs::PoseStamped& start,
                       const geometry_msgs::PoseStamped& goal);
    
    bool checkCollision(const geometry_msgs::Pose& pose, float radius = 1.0);
    float computeClearance(const geometry_msgs::Pose& pose);
    
    void publishMapVisualization();
    void publishPathVisualization();
    void publishRemapStatus();
    
    void run();
};

#endif
