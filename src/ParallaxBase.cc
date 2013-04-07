#include "ParallaxBase.h"

#include "VelocityController.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <parallax_base/RawData.h>
#include <cmath>

const double ParallaxBase::PI = 3.1415926536;
const double ParallaxBase::DEFAULT_WHEEL_RADIUS = 6 * 2.54 * 0.01; // 6" wheel
const double ParallaxBase::DEFAULT_AXLE_LENGTH = 24 * 2.54 * 0.01; // 24" axle length
const double ParallaxBase::DEFAULT_WHEEL_ROTATION_SPEED = 2.5 * 2 * PI; // Rotations per second
const double ParallaxBase::DEFAULT_WHEEL_ACCEL = (4 * 15.0 / 144) * 2 * PI; // Rotations per second squared
const double ParallaxBase::DEFAULT_RATE = 30.0;

ParallaxBase::ParallaxBase() : m_n("~") {
    // get parameters
    m_n.param<std::string>("portname", m_portname, "/dev/ttyUSB0");
    m_n.param<std::string>("nodename", m_nodename, "parallax_base");
    m_n.param<std::string>("framename", m_framename, "parallax_base_frame");
    m_n.param<std::string>("odomframe", m_odomframe, "odom");

    m_n.param<double>("rate", m_rate, DEFAULT_RATE);
    m_n.param<double>("wheel_radius", m_wheel_radius, DEFAULT_WHEEL_RADIUS);
    m_n.param<double>("wheel_speed", m_max_wheel_speed, DEFAULT_WHEEL_ROTATION_SPEED);
    m_n.param<double>("wheel_accel", m_wheel_accel, DEFAULT_WHEEL_ACCEL);
    m_n.param<double>("axle_length", m_axle_length, DEFAULT_AXLE_LENGTH);

    m_n.param<int>("left_motor_id", m_left_motor_id, 1);
    m_n.param<int>("right_motor_id", m_right_motor_id, 2);

    m_n.param<bool>("publish_tf", m_publish_tf, true);

    // units: m/s
    m_n.param<double>("max_linear_speed", m_max_linear_speed, 
            m_wheel_radius * m_max_wheel_speed);
    // units: rad/s
    m_n.param<double>("max_rotational_speed", m_max_rotational_speed,
            m_max_linear_speed / (PI * m_axle_length));

    // set up publishers
    m_odometry_publisher = m_n.advertise<nav_msgs::Odometry>(m_nodename + "/wheel_odom", 1);
    m_auxiliary_data_publisher = m_n.advertise<parallax_base::RawData>(m_nodename + "/aux_data", 1);

    // set up subscribers
    m_twist_subscriber = m_n.subscribe<geometry_msgs::Twist>(m_nodename + "/cmd_vel",
            10, &ParallaxBase::twistCallback, this);

    // initialize variables
    m_seq = 0;

    uint8_t ids[] = {m_left_motor_id, m_right_motor_id};
    m_control = new VelocityController(m_portname, ids, 2, m_rate);
    m_control->setAccelRadians(m_left_motor_id, m_wheel_accel / (2 * PI));
    m_control->setAccelRadians(m_right_motor_id, m_wheel_accel / (2 * PI));
    m_control->setSpeedRadians(m_left_motor_id, 0.0);
    m_control->setSpeedRadians(m_right_motor_id, 0.0);

    // rawdata
    fillHeader(&m_rawdata.header);
    m_rawdata.left_enc_pos = 0;
    m_rawdata.right_enc_pos = 0;
    m_rawdata.left_enc_spd = 0;
    m_rawdata.right_enc_spd = 0;
    m_rawdata.left_req_speed = 0;
    m_rawdata.right_req_speed = 0;
    m_odom_pos_x = 0;
    m_odom_pos_y = 0;
    m_odom_pos_yaw = 0;
    m_odom_vel_x = 0;
    m_odom_vel_y = 0;
    m_odom_vel_yaw = 0;

    m_timer = m_n.createTimer(ros::Rate(m_rate), &ParallaxBase::backgroundTask, this);
    m_timer.start();
}

ParallaxBase::~ParallaxBase() {
    m_timer.stop();
    delete m_control;
}

#ifndef SIGN
#define SIGN(X) ((X < 0) ? (-1) : (1))
#endif

void ParallaxBase::twistCallback(const geometry_msgs::Twist::ConstPtr& twist) {
    ROS_DEBUG("Velocity command given: [linear: %.03f m/s, rotational: %.03f m/s]", 
           twist->linear.x, twist->angular.z);
    double linear = twist->linear.x;
    double angular = twist->angular.z;

    if (abs(linear) > m_max_linear_speed) {
        linear = SIGN(linear) * m_max_linear_speed;
        ROS_WARN("Clamping linear speed to %.03f m/s", linear);
    }

    if (abs(angular) > m_max_rotational_speed) {
        angular = SIGN(angular) * m_max_rotational_speed;
        ROS_WARN("Clamping angular sped to %.03f rad/s", angular);
    }

    double left_req = linear - m_axle_length * angular;
    double right_req = linear + m_axle_length * angular;

    if (abs(left_req) > m_max_wheel_speed) {
        left_req = SIGN(left_req) * m_max_wheel_speed;
        ROS_WARN("Clamping left wheel speed to %.03f", left_req);
    }

    if (abs(right_req) > m_max_wheel_speed) {
        right_req = SIGN(right_req) * m_max_wheel_speed;
        ROS_WARN("Clamping right wheel speed to %.03f", right_req);
    }

    ROS_DEBUG("Requesting wheel speeds [%.03f, %.03f]", left_req, right_req);

    m_rawdata.left_req_speed = left_req;
    m_rawdata.right_req_speed = right_req;
}

#undef SIGN

void ParallaxBase::fillHeader(std_msgs::Header * header) {
    header->seq = m_seq;
    header->stamp = ros::Time::now();
    header->frame_id = m_framename;
}

#ifndef NORMALIZE
#define NORMALIZE(Z) (atan2(sin(Z), cos(Z)))
#endif

void ParallaxBase::backgroundTask(const ros::TimerEvent& e) {
    m_control->setSpeedRadians(m_left_motor_id, m_rawdata.left_req_speed / (2 * PI * m_wheel_radius));
    m_control->setSpeedRadians(m_right_motor_id, m_rawdata.right_req_speed / (2 * PI * m_wheel_radius));

    m_control->backgroundTask();

    // update rawdata
    fillHeader(&m_rawdata.header);
    m_rawdata.left_enc_spd = m_control->getSpeedRadians(m_left_motor_id) * 2 * PI * m_wheel_radius;
    m_rawdata.right_enc_spd = m_control->getSpeedRadians(m_right_motor_id) * 2 * PI * m_wheel_radius;
    m_rawdata.left_enc_pos = m_control->getPositionTicks(m_left_motor_id);
    m_rawdata.right_enc_pos = m_control->getPositionTicks(m_right_motor_id);
    m_auxiliary_data_publisher.publish(m_rawdata);

    // update odom
    double dt = 1.0 / m_rate;
    double dl = m_rawdata.left_enc_spd * dt;
    double dr = m_rawdata.right_enc_spd * dt;
    double dth = (dr - dl) / m_axle_length;
    double dv = (dr + dl) / 2;

    m_odom_vel_yaw = NORMALIZE(dth / dt);
    m_odom_pos_yaw = NORMALIZE(m_odom_pos_yaw + dth);

    double dx = dv * cos(m_odom_pos_yaw);
    double dy = dv * sin(m_odom_pos_yaw);

    m_odom_vel_x = dx / dt;
    m_odom_vel_y = dy / dt;

    m_odom_pos_x += dx;
    m_odom_pos_y += dy;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_odom_pos_yaw);

    if (m_publish_tf) {
        // create a tf message
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.seq = m_seq;
        odom_trans.header.frame_id = m_odomframe;
        odom_trans.child_frame_id = m_framename;
        
        odom_trans.transform.translation.x = m_odom_pos_x;
        odom_trans.transform.translation.y = m_odom_pos_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        m_tf_broadcaster.sendTransform(odom_trans);
    }

    // create an odom message
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.seq = m_seq;
    odom.header.frame_id = m_odomframe;

    odom.pose.pose.position.x = m_odom_pos_x;
    odom.pose.pose.position.y = m_odom_pos_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = m_framename;
    odom.twist.twist.linear.x = m_odom_vel_x;
    odom.twist.twist.linear.y = m_odom_vel_y;
    odom.twist.twist.angular.z = m_odom_vel_yaw;

    double covariance[36] = { 
        1e-3, 0, 0, 0, 0, 0, 
        0, 1e-3, 0, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0, 
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e3 
    };

    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = covariance[i];
        odom.twist.covariance[i] = covariance[i];
    }

    m_odometry_publisher.publish(odom);

    m_seq++;
}

#undef NORMALIZE
