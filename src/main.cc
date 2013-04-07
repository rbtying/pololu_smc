#include <ros/ros.h>
#include "ParallaxBase.h"

/**
 * Main runner function for the node
 */
int main(int argc, char** argv) {
    // init ros
    ros::init(argc, argv, "parallax_base");
    ParallaxBase base;
    ros::spin();
}
