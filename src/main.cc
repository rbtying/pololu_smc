#include <ros/ros.h>
#include "SMCNode.h"

/**
 * Main runner function for the node
 */
int main(int argc, char** argv) {
    // init ros
    ros::init(argc, argv, "smc_base");
    SMCNode s;
    ros::spin();
}
