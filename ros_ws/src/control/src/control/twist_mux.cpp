/**
 * @file twist_mux.cpp
 * @class TwistMux
 * @brief The implementation file for the TwistMux class
 */

#include <twist_mux.h>

TwistMux::TwistMux()
{
    //Ensure that the robot starts in a safe standby mode
    current_mode = standby;
    stopRobot();

    //Setup publishers and subscribers
    joy_sub = nh.subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, &TwistMux::joyCallback, this);
    teleop_sub = nh.subscribe<geometry_msgs::Twist>(TELEOP_TOPIC, 1, &TwistMux::twistCallback, this);
    autonomous_sub = nh.subscribe<geometry_msgs::Twist>(AUTO_TOPIC, 1, &TwistMux::twistCallback, this);

    twist_pub = nh.advertise<geometry_msgs::Twist>(CONTROL_TOPIC, 1);
}

void TwistMux::update()
{

}

void TwistMux::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if(msg->buttons[B_BUTTON] == 1)
        current_mode = standby;
    else if(msg->buttons[A_BUTTON] == 1)
        current_mode = teleop;
    else if(msg->buttons[Y_BUTTON] == 1)
        current_mode = autonomous;
}

void TwistMux::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.linear.y = msg->linear.y;
    cmd_vel.linear.z = msg->linear.z;
    cmd_vel.angular.x = msg->angular.x;
    cmd_vel.angular.y = msg->angular.y;
    cmd_vel.angular.z = msg->angular.z;
}

void TwistMux::stopRobot()
{
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
}

