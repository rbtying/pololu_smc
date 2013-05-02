#include "SMCNode.h"

#include <serial/serial.h>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include "PololuSMC.h"

#include <ros/ros.h>

#include <std_msgs/Float64.h>

const double DEFAULT_RATE = 50.0;
const double PI = 3.1415926535;

SMCNode::SMCNode() : m_n("~") {
    std::string id_list;    
    m_n.getParam("id_list", id_list);
    std::istringstream iss(id_list);
    int val;
    while (iss >> val) {
        m_ids.push_back(static_cast<uint8_t>(val));
    }

    std::string portname;
    m_n.param<std::string>("portname", portname, "/dev/ttyUSB0");

    double defaultp, defaulti, defaultd;
    m_n.param<double>("default_p", defaultp, 1);
    m_n.param<double>("default_i", defaulti, 0);
    m_n.param<double>("default_d", defaultd, 0);

    m_serial = new serial::Serial(portname, 1000000);

    m_controllers.resize(m_ids.size(), NULL);
    m_pubs.resize(m_ids.size());
    m_vpubs.resize(m_ids.size());
    m_subs.resize(m_ids.size());
    m_pids.resize(m_ids.size());
    m_min_val.resize(m_ids.size());
    m_max_val.resize(m_ids.size());
    m_vcenter_val.resize(m_ids.size());

    for (int i = 0; i < m_ids.size(); i++) {
        PololuSMC *controller = new PololuSMC(m_serial, m_ids[i]);
        m_controllers[i] = controller;
        m_controllers[i]->open();

        int id = m_ids[i];

        std::string base = "/smc_" + boost::lexical_cast<std::string>(id);
        ROS_WARN("%s, %d", base.c_str(), m_ids[i]);
        double kp, ki, kd;
        m_n.param<double>(base + "_p", kp, defaultp);
        m_n.param<double>(base + "_i", ki, defaulti);
        m_n.param<double>(base + "_d", kd, defaultd);
        m_pids[i].setPID(kp, ki, kd);

        double min, max;
        m_n.param<double>(base + "_min", min, -1.57);
        m_n.param<double>(base + "_max", max, 1.57);
        m_min_val[i] = min;
        m_max_val[i] = max;

        double vcenter;
        m_n.param<double>(base + "_vcenter", vcenter, 1.65);
        m_vcenter_val[i] = vcenter;

        m_pubs[i] = m_n.advertise<std_msgs::Float64>(base + "/current_pos", 1);
        m_vpubs[i] = m_n.advertise<std_msgs::Float64>(base + "/voltage", 1);
        m_subs[i] = m_n.subscribe<std_msgs::Float64>(base + "/desired_pos", 1, boost::bind(&SMCNode::pos_cb, this, _1, m_ids[i]));
    }

    m_enabled = 0;
    m_enable_subscriber = m_n.subscribe<std_msgs::Bool>("/robot_enabled", 1, &SMCNode::enable_cb, this);

    m_timer = m_n.createTimer(ros::Rate(DEFAULT_RATE), &SMCNode::backgroundTask, this);
    m_timer.start();
}

SMCNode::~SMCNode() {
    m_timer.stop();

    for (int i = 0; i < m_controllers.size(); i++) {
        m_controllers[i]->setMotorSpeed(0.0);
        delete m_controllers[i];
    }

    m_serial->close();
    delete m_serial;
}

int SMCNode::lookup_id(int id) {
    for (int i = 0; i < m_ids.size(); i++) {
        if (id == m_ids[i]) {
            return i;
        }
    }
    ROS_ERROR("SMC id %d not found", id);
    return -1;
}

void SMCNode::pos_cb(const std_msgs::Float64::ConstPtr& data, uint8_t id) {
    ROS_INFO("%d: %f", id, data->data);
    m_pids[lookup_id(id)].setSetpoint(data->data);
}

void SMCNode::enable_cb(const std_msgs::Bool::ConstPtr& data) {
    m_enabled = data->data;
}

void SMCNode::backgroundTask(const ros::TimerEvent& e) {
    double dt = 1 / DEFAULT_RATE;

    for (int i = 0; i < m_ids.size(); i++) {
        try {
            double input = m_controllers[i]->getAN1();
            double angle = (input - m_vcenter_val[i]) / 3.3 * 5 * 2 * PI;

            if (input < 0) {
                angle = 0;
            }

            std_msgs::Float64 ang;
            ang.data = angle;
            m_pubs[i].publish(ang);

            std_msgs::Float64 volt;
            volt.data = m_controllers[i]->getVoltage();
            m_vpubs[i].publish(volt);

            m_pids[i].update(angle, dt);
            /* m_controllers[i]->setMotorSpeed(m_pids[i].getOutput()); */

            double output = m_pids[i].getOutput();

            if (m_enabled) {
                m_controllers[i]->setMotorSpeed(output);
            } else {
                m_controllers[i]->setMotorSpeed(0.0);
            }

        } catch (std::exception& e) {
            m_controllers[i]->setMotorSpeed(0.0);
            ROS_WARN("Exception: %s",  e.what());
        }
    }
}
