#include "radar_obstacle_avoidance/rtk_gps_parser.h"
using namespace radar_obstacle_avoidance;

int main(int argc, char** argv) {
    ros::init(argc, argv, "rtk_gps_node");
    RTKGPSParser parser;
    parser.run();
    return 0;
}
