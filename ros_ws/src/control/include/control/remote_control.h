/**
 * @file remote_control.h
 * @class RemoteControl
 * @brief The class which converts joy messages to twist
 *
 * This class will act as a way to take controller inputs and output them to a format which is usable by the robot.  It
 * is currently setup to accept joy messages and output them as a twist message which only sets the linear.x and
 * angular.z velocities.
 */

#ifndef CONTROL_INCLUDE_CONTROL_REMOTE_CONTROL_H_
#define CONTROL_INCLUDE_CONTROL_REMOTE_CONTROL_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "constants.h"

class RemoteControl
{
private:
    /**
     * @brief The Twist message to be published
     *
     * The twist message is a set of velocities to be sent to the motor controllers.  It will only use the linear.x and
     * angular.z fields since the robot will only be able to accelerate in these ways.
     */
    geometry_msgs::Twist cmd_vel;

    /**
     * @brief The twist publisher
     *
     * This ros publisher will be responsible for publishing the twist message which will eventually be sent to the
     * robot's motor controllers.
     */
    ros::Publisher twist_pub;
    /**
     * @brief The joy message subscriber
     *
     * This ros subscriber will be responsible for subscribing to the joy message published by the controller method that
     * is being used.
     */
    ros::Subscriber joy_sub;
    /**
     * @brief The ros node handle
     */
    ros::NodeHandle nh;

    /**
     * @brief The speed modifier. Represents a percentage of max speed
     */
    double speed_modifier;

    /**
     * @brief The callback function for joy messages
     *
     * This function accepts joy messages and converts them into a twist format in the form of cmd_vel.  It will change
     * the linear.x and angular.z velocities of the cmd_vel variable.
     *
     * @param msg The message which is received from the joy publisher
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

public:
    /**
     * @brief The RemoteControl default constructor
     *
     * The RemoteControl constructor initializes the RemoteControl class by setting the initial value of cmd_vel to all
     * 0s.  It also initializes the subscriber joy_sub to subscribe to the topic "joy" and the publisher twist_pub to
     * publish to the topic "teleop_twist".
     */
    RemoteControl();

    /**
     * @brief The RemoteControl constructor
     *
     * This constructor allows the user to set the values of the topics to subscribe and publish to.
     *
     * @param twist_topic The topic to publish the twist message to
     * @param joy_topic The topic to subscribe to for the joy messages
     */
    RemoteControl(std::string twist_topic, std::string joy_topic);

    /**
     * @brief Updates the message which is being published
     *
     * This function is used to send updated messages to any subscribers.  It provides a way for the publish method to be
     * called outside of the class.
     */
    void update();
};

#endif /* CONTROL_INCLUDE_CONTROL_REMOTE_CONTROL_H_ */
