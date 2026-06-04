#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudProcessor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    float voxel_size_;

public:
    PointCloudProcessor() : nh_("~") {
        nh_.param("voxel_size", voxel_size_, 0.1f);

        cloud_sub_ = nh_.subscribe("/lidar_points", 5,
                                   &PointCloudProcessor::cloudCallback, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_points", 5);

        ROS_INFO("PointCloudProcessor initialized, voxel size: %.2f", voxel_size_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        // Apply voxel grid filter
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*filtered);

        // Convert back to ROS message and publish
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered, output);
        output.header = msg->header;
        cloud_pub_.publish(output);

        ROS_DEBUG("Published filtered point cloud with %zu points", filtered->size());
    }

    void run() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_processor");
    PointCloudProcessor processor;
    processor.run();
    return 0;
}
