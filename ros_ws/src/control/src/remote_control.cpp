/**
 * @file RemoteControl.cpp
 * @brief The implementation file for the RemoteControl class
 *
 * This file implements the functions defined in RemoteControl.h
 */

#include "remote_control.h"

RemoteControl::RemoteControl()
{
  // Initialize cmd_vel to 0
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;

  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &RemoteControl::joyCallback,
                                           this);
  twist_pub = nh.advertise<geometry_msgs::Twist>("teleop_twist", 1);
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

    joy_sub = nh.subscribe<sensor_msgs::Joy>(twist_topic, 1, &RemoteControl::joyCallback,
                                             this);
    twist_pub = nh.advertise<geometry_msgs::Twist>(joy_topic, 1);
}

void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  cmd_vel.linear.x = msg->axes[1];
  cmd_vel.angular.z = msg->axes[2];
}

void RemoteControl::update()
{
  twist_pub.publish(cmd_vel);
}
