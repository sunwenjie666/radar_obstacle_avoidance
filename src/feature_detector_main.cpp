#include "radar_obstacle_avoidance/feature_detector.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "feature_detector_node");
    
    FeatureDetector detector;
    detector.run();
    
    return 0;
}
