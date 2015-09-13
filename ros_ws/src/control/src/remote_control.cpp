/**
 * @file RemoteControl.cpp
 * @brief The implementation file for the RemoteControl class
 *
 * This file implements the functions defined in RemoteControl.h
 */

#include "remote_control.h"

const int Y_AXIS_L_STICK = 1;
const int X_AXIS_R_STICK = 2;

const std::string JOY_TOPIC = "joy";
const std::string TELEOP_TOPIC = "teleop_twist";

RemoteControl::RemoteControl()
{
    // Initialize cmd_vel to 0
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;

    joy_sub = nh.subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, &RemoteControl::joyCallback, this);
    twist_pub = nh.advertise<geometry_msgs::Twist>(TELEOP_TOPIC, 1);
}

RemoteControl::RemoteControl(std::string twist_topic, std::string joy_topic)
{
    // Initialize cmd_vel to 0
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;

    joy_sub = nh.subscribe<sensor_msgs::Joy>(twist_topic, 1, &RemoteControl::joyCallback, this);
    twist_pub = nh.advertise<geometry_msgs::Twist>(joy_topic, 1);
}

void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    cmd_vel.linear.x = msg->axes[Y_AXIS_L_STICK];
    cmd_vel.angular.z = msg->axes[X_AXIS_R_STICK];
}

void RemoteControl::update()
{
    twist_pub.publish(cmd_vel);
}
