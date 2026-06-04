#include "radar_obstacle_avoidance/remap_manager.h"

using namespace radar_obstacle_avoidance; 
int main(int argc, char** argv) {
    ros::init(argc, argv, "remap_manager_node");
    RemapManager manager;
    manager.run();
    return 0;
}
