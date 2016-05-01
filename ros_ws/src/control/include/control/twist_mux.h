/**
 * @file twist_mux.h
 * @class TwistMux
 * @brief A simple class which accepts multiple twist messages and outputs only one
 *
 * This class should act as a way to select which twist message is being sent to the robots motor controllers.  The
 * message can come from multiple places including the teleop node, the estop node (built in to this class), or the
 * autonomous node.  The twist mux will use joy messages to decide which twist message should be used at any given
 * time.
 */

#ifndef CONTROL_INCLUDE_CONTROL_TWIST_MUX_H_
#define CONTROL_INCLUDE_CONTROL_TWIST_MUX_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "constants.h"

/**
 * @brief Control namespace
 */
namespace Control
{
    /**
     * @brief An enumerator to describe the mode the robot is currently in
     */
    enum Mode
    {
        standby,        //!< The robot is not moving, the same as a software e-stop
        teleop,         //!< The robot is currently being controlled by remote control
        autonomous      //!< The robot is driving itself
    };
}

class TwistMux
{
private:
    /**
     * @brief The mode which the robot is currently in
     */
    Control::Mode current_mode;

    /**
     * @brief The twist message which is sent to the motor controllers of the robot
     */
    geometry_msgs::Twist cmd_vel;

    /**
     * @brief The joy message subscriber
     *
     * This ros subscriber will be responsible for subscribing to the joy message published by the controller method
     * that is being used.
     */
    ros::Subscriber joy_sub;

    /**
     * @brief The subscriber which receives twist messages from the teleop node
     */
    ros::Subscriber twist_sub;

    /**
     * @brief The twist message publisher
     *
     * This ros publisher is responsible for the publishing of the twist message which will control the robot.
     */
    ros::Publisher twist_pub;

    /**
     * @brief The Status Light message publisher
     *
     * This ros publisher is responsible for the publishing of the Status Light Message
     */
    ros::Publisher light_pub;

    /**
     * @brief The ros node handle
     */
    ros::NodeHandle nh;

    /**
     * @brief The callback function for joy messages
     *
     * This function takes joy messages and reads information from the buttons to decide what mode it should currently
     * be running in.
     *
     * @param msg The message which is received from the joy publisher
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

    /**
     * @brief The callback function for twist messages
     *
     * This function takes in twist messages and parses the information to the member variable cmd_vel which is
     * published to the motor controllers.
     *
     * @param msg The message which is received from the twist publisher
     */
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    /**
     * @brief Allows config values to be used. Ex: config.max_speed
     */
    ConfigValues config;
    /**
     * @brief Sets all the fields in cmd_vel to 0 to ensure the robot is not moving
     */
    void stopRobot();

public:
    TwistMux();

    void update();
};

#endif /* CONTROL_INCLUDE_CONTROL_TWIST_MUX_H_ */
