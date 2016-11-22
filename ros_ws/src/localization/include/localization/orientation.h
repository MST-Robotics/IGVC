/**
 * @file orientation.h
 * @class Orientation
 * @brief The class definition file for the Orientation class
 */

#ifndef PROJECT_ORIENTATION_H
#define PROJECT_ORIENTATION_H

#include <ros/ros.h>
#include <localization/rpy.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

class Orientation
{
    private:
        /**
         * @brief The ros nodehandle
         */
        ros::NodeHandle nh;

        /**
         * @brief The subscriber to the message from the imu, expects an rpy message
         */
        ros::Subscriber imu_sub;

        /**
         * @brief The publisher of the imu message
         */
        ros::Publisher orientation_pub;

        /**
         * @brief The orientation of the robot
         */
        sensor_msgs::Imu orientation;

        /**
         * @brief The callback function for the rpy message
         *
         * This function will accept rpy messages and will publish this information to an imu message
         *
         * @param msg The message received from the rpy publisher
         */
        void rpyCallback(const localization::rpy::ConstPtr& msg);

    public:
        Orientation();

        void update();
};


#endif //PROJECT_ORIENTATION_H
