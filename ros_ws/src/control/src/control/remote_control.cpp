/**
 * @file remote_control.cpp
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
    speed_modifier = 0.5;
    
    //Initilize the config values
    config = initConfigs();
    
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
    speed_modifier = 0.5;

    joy_sub = nh.subscribe<sensor_msgs::Joy>(twist_topic, 1, &RemoteControl::joyCallback, this);
    twist_pub = nh.advertise<geometry_msgs::Twist>(joy_topic, 1);
}

void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

    if ((msg->buttons[config.speed_inc_btn] == 1) && (speed_modifier < 1))
    {
        speed_modifier += 0.01;
    }
    else if ((msg->buttons[config.speed_dec_btn] == 1) && (speed_modifier > 0))
    {
        speed_modifier -= 0.01;
    }
    cmd_vel.linear.x = msg->axes[config.lin_vel_axis] * speed_modifier;
    cmd_vel.angular.z = msg->axes[config.ang_vel_axis];
}

void RemoteControl::update()
{
    twist_pub.publish(cmd_vel);
}
