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
#include "constants.h"

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
     * @brief The right velocity to be published
     */
    std_msgs::Float64 right_vel;

    /**
     * @brief The left velocity to be published
     */
    std_msgs::Float64 left_vel;

    /**
     * @brief The twist message subscriber
     */
    ros::Subscriber twist_sub;

    /**
     * @brief The right wheel velocity publisher
     */
    ros::Publisher right_vel_pub;

    /**
     * @brief The left wheel velocity publisher
     */
    ros::Publisher left_vel_pub;

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
