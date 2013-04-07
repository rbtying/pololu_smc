#ifndef PARALLAX_BASE_H_
#define PARALLAX_BASE_H_

// libraries
#include <string>

// ros
#include <ros/ros.h>

// messages
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <parallax_base/RawData.h>
#include <tf/transform_broadcaster.h>

class VelocityController;

/**
 * Class to implement a ParallaxBase node, which uses the Parallax Position
 * Controller driver to make a differential-drive robot
 */
class ParallaxBase {
    public:
        /**
         * Constructs a ParallaxBase node
         */
        ParallaxBase();

        /**
         * Destructs a ParallaxBase node
         */
        ~ParallaxBase();
    private:
        /**
         * Callback function for twist message processing
         * @param twist the message to process
         */
        void twistCallback(const geometry_msgs::Twist::ConstPtr& twist);

        /**
         * Fills the header with the current timestamp, frame, and seq
         * @param header a pointer to the header to fill
         */
        void fillHeader(std_msgs::Header * header);

        /**
         * Runs in the background at rate m_rate to process and communicate with
         * the controllers.
         * @param e the timerevent associated with this partiular call
         */
        void backgroundTask(const ros::TimerEvent& e);

        // ROS handle
        ros::NodeHandle m_n;

        // Publishers
        ros::Publisher m_odometry_publisher;
        ros::Publisher m_auxiliary_data_publisher;
        tf::TransformBroadcaster m_tf_broadcaster;

        // Subscribers
        ros::Subscriber m_twist_subscriber;

        // Parameters
        std::string m_nodename;
        std::string m_framename;
        std::string m_portname;
        std::string m_odomframe;
        double m_max_linear_speed;
        double m_max_rotational_speed;

        double m_wheel_radius;
        double m_wheel_accel;
        double m_max_wheel_speed;
        double m_axle_length;
        double m_rate;
        int m_left_motor_id;
        int m_right_motor_id;

        bool m_publish_tf;

        // constants
        static const double DEFAULT_WHEEL_RADIUS;
        static const double DEFAULT_AXLE_LENGTH;
        static const double DEFAULT_WHEEL_ROTATION_SPEED;
        static const double DEFAULT_WHEEL_ACCEL;
        static const double PI;
        static const double DEFAULT_RATE;

        // Other members
        parallax_base::RawData m_rawdata;
        uint32_t m_seq;
        ros::Timer m_timer;

        double m_odom_pos_x;
        double m_odom_pos_y;
        double m_odom_pos_yaw;
        double m_odom_vel_x;
        double m_odom_vel_y;
        double m_odom_vel_yaw;

        VelocityController * m_control;
};

#endif /* PARALLAX_BASE_H_ */
