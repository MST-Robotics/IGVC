/**
 * @file orientation.cpp
 * @class Orientation
 * @brief The class implementation file for the orientation class
 */
#include "orientation.h"

Orientation::Orientation()
{
    // Initialize the imu subscriber
    imu_sub = nh.subscribe<localization::rpy>("imu", 1, &Orientation::rpyCallback, this);
    orientation_pub = nh.advertise<sensor_msgs::Imu>("orientation", 1);
}

void Orientation::rpyCallback(const localization::rpy::ConstPtr &msg)
{
    orientation.header.stamp = ros::Time::now();
    orientation.header.frame_id = "camera_link";
    orientation.orientation = tf::createQuaternionMsgFromRollPitchYaw(msg->roll, msg->pitch, msg->yaw);
}

void Orientation::update()
{
    orientation_pub.publish(orientation);
}

