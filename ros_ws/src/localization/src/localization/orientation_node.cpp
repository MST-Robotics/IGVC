/**
 * @file orientation_node.cpp
 * @brief The orientation node source file
 */

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orientation_node");

    // Spin at 30hz
    ros::Rate loop_rate(30);

    // Spin forever as long as ros is okay
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}