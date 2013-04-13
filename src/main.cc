#include <ros/ros.h>

/**
 * Main runner function for the node
 */
int main(int argc, char** argv) {
    // init ros
    ros::init(argc, argv, "parallax_base");
    ros::spin();
}
