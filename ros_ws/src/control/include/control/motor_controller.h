/**
 * @file motor_controller.h
 * @class MotorController
 * @brief The class which converts twist messages to angular velocities for the robot's wheels
 */

#ifndef CONTROL_INCLUDE_CONTROL_MOTOR_CONTROLLER_H_
#define CONTROL_INCLUDE_CONTROL_MOTOR_CONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "constants.h"

/* Custom messages - This file should be Auto generated */
#include "control/Encoder.h"

class MotorController
{
private:
    /**
     * @brief The diameter of the robot from one wheel to another
     */
    double robot_base;

    /**
     * @brief The radius of the wheels
     */
    double wheel_rad;

    /**
     * @brief The maximum speed of the robot
     */
    double max_speed;

    /**
     * @brief The unscaled maximum speed of the wheel velocity function
     */
    double unscaled_max_speed;

    /**
     * @brief The encoder resolution in ticks/s
     */
    double encoder_res;

    /**
     * @brief The right velocity to be published
     */
    std_msgs::Float64 right_vel;

    /**
     * @brief The left velocity to be published
     */
    std_msgs::Float64 left_vel;

    /**
     * @brief The linear displacement of the left wheel since the last message
     */
    double displacement_left;

    /**
     * @brief The linear displacement of the right wheel since the last message
     */
    double displacement_right;

    /**
     * @brief The previous angular displacement of the robot
     */
    double prev_theta;

    /**
     * @brief The tf transform to be published specifying odometry
     */
    geometry_msgs::TransformStamped odom_trans;

    /**
     * @brief The tf broadcaster for the odom transform
     */
    tf::TransformBroadcaster odom_broadcaster;

    /**
     * @brief The twist message subscriber
     */
    ros::Subscriber twist_sub;

    /**
     * @brief The joint state message to be published containing the left and right wheel positions
     */
    sensor_msgs::JointState joint_state;

    /**
     * @brief The subscriber for the left encoder
     */
    ros::Subscriber left_encoder;

    /**
     * @brief The subscriber for the right encoder
     */
    ros::Subscriber right_encoder;

    /**
     * @brief The right wheel velocity publisher
     */
    ros::Publisher right_vel_pub;

    /**
     * @brief The left wheel velocity publisher
     */
    ros::Publisher left_vel_pub;

    /**
     * @brief The joint state publisher for the left and right wheel positions
     */
    ros::Publisher joint_state_pub;

    /**
     * @brief The ros node handle
     */
    ros::NodeHandle nh;

    /**
     * @brief The callback function for twist messages
     *
     * This function accepts twist messages and converts them into an angular velocity in the form of a Float64 which is
     * sent to motor controller board
     *
     * @param msg The message which is received from the twist publisher
     */
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /**
    * @brief The callback function for the right encoder message
    *
    * This function accepts Encoder messages and records the number of ticks in a time interval for the left wheel
    *
    * @param msg The message which is received from the right encoder publisher
    */
    void leftEncoderCallback(const control::Encoder::ConstPtr& msg);

    /**
     * @brief The callback function for the right encoder message
     *
     * This function accepts Encoder messages and records the number of ticks in a time interval for the right wheel
     *
     * @param msg The message which is received from the right encoder publisher
     */
    void rightEncoderCallback(const control::Encoder::ConstPtr& msg);

    /**
     * @brief Calculates the right wheel angular velocity based on a differential drive model
     * @param lin_vel The robot's linear velocity
     * @param ang_vel The robot's angular velocity
     * @return The right wheel's angular velocity
     */
    double getRightVel(const double lin_vel, const double ang_vel);

    /**
     * @brief Calculates the left wheel angular velocity based on a differential drive model
     * @param lin_vel The robot's linear velocity
     * @param ang_vel The robot's angular velocity
     * @return The left wheel's angular velocity
     */
    double getLeftVel(const double lin_vel, const double ang_vel);

    /**
     * @brief Scales a given double range to a new given double range
     * @param val The value to scale
     * @param pre_min The minimum value before scaling
     * @param pre_max The maximum value before scaling
     * @param scale_min The minimum value after scaling
     * @param scale_max The maximum value after scaling
     * @return The scaled value
     */
    double scale(const double val, const double pre_min, const double pre_max, const double scale_min,
                 const double scale_max);

public:
    /**
     * @brief The MotorController default constructor
     *
     * The default constructor sets up the class to subscribe to the twist topic CONTROL_TOPIC and publish to the float
     * topics LEFT_VEL_TOPIC and RIGHT_VEL_TOPIC which are defined in the constants.h file.
     */
    MotorController();

    /**
     * @brief Publish updates to the motors
     */
    void update();
};

#endif /* CONTROL_INCLUDE_CONTROL_MOTOR_CONTROLLER_H_ */
