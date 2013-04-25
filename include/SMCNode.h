#ifndef PARALLAX_BASE_H_
#define PARALLAX_BASE_H_

// libraries
#include <string>
#include <vector>
#include "PololuSMC.h"
#include "PID.h"

// ros
#include <ros/ros.h>

// messages
#include <std_msgs/Float64.h>

/**
 * Class to implement a SMCNode node.
 */
class SMCNode {
    public:
        /**
         * Constructs a SMCNode node
         */
        SMCNode();

        /**
         * Destructs a SMCNode node
         */
        ~SMCNode();
    private:
        /**
         * Runs in the background at rate m_rate to process and communicate with
         * the controllers.
         * @param e the timerevent associated with this partiular call
         */
        void backgroundTask(const ros::TimerEvent& e);

        /**
         * Callback that handles float64 messages sent to this node
         *
         * @param data the data being sent
         * @param id the id of the node
         */
        void pos_cb(const std_msgs::Float64::ConstPtr& data, uint8_t id);

        int lookup_id(int id);

        // ROS handle
        ros::NodeHandle m_n;
        ros::Timer m_timer;

        serial::Serial *m_serial;

        std::vector<uint8_t> m_ids;
        std::vector<PololuSMC*> m_controllers;
        std::vector<double> m_min_val;
        std::vector<double> m_max_val;
        std::vector<double> m_vcenter_val;
        std::vector<PID> m_pids;
        std::vector<ros::Publisher> m_pubs;
        std::vector<ros::Publisher> m_vpubs;
        std::vector<ros::Subscriber> m_subs;
};

#endif /* PARALLAX_BASE_H_ */
