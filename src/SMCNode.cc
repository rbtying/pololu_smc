#include "SMCNode.h"

#include <XmlRpcValue.h>
#include <serial/serial.h>
#include <string>
#include "PololuSMC.h"

#include <ros/ros.h>

#include <std_msgs/Float64.h>

const double DEFAULT_RATE = 50.0;
const double PI = 3.1415926535;

SMCNode::SMCNode() : m_n("~") {
    XmlRpc::XmlRpcValue id_list;    
    m_n.getParam("id_list", id_list);
    ROS_ASSERT(id_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < id_list.size(); i++) {
        ROS_ASSERT(id_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        int val = id_list[i];
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
    m_subs.resize(m_ids.size());
    m_pids.resize(m_ids.size());
    for (int i = 0; i < m_ids.size(); i++) {
        PololuSMC *controller = new PololuSMC(m_serial, m_ids[i]);
        m_controllers[i] = controller;
        m_controllers[i]->open();

        std::string base = "smc_" + boost::lexical_cast<std::string>(m_ids[i]);
        double kp, ki, kd;
        m_n.param<double>(base + "_p", kp, defaultp);
        m_n.param<double>(base + "_i", ki, defaulti);
        m_n.param<double>(base + "_d", kd, defaultd);
        m_pids[i].setPID(kp, ki, kd);

        m_pubs[i] = m_n.advertise<std_msgs::Float64>(base + "/current_pos", 1);
        m_subs[i] = m_n.subscribe<std_msgs::Float64>(base + "/desired_pos", 1, boost::bind(&SMCNode::pos_cb, this, _1, m_ids[i]));
    }

    m_timer = m_n.createTimer(ros::Rate(DEFAULT_RATE), &SMCNode::backgroundTask, this);
    m_timer.start();
}

SMCNode::~SMCNode() {
    m_timer.stop();

    for (int i = 0; i < m_controllers.size(); i++) {
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

void SMCNode::backgroundTask(const ros::TimerEvent& e) {
    double dt = 1 / DEFAULT_RATE;

    for (int i = 0; i < m_ids.size(); i++) {
        double input = (m_controllers[i]->getAN1() - 1.65) / 3.3 * 5 * 2 * PI;
        m_pids[i].update(input, dt);
        m_controllers[i]->setMotorSpeed(m_pids[i].getOutput());
        std_msgs::Float64 ang;
        ang.data = input;
        m_pubs[i].publish(ang);
    }
}
