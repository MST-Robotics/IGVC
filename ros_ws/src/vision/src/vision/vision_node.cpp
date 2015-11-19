/**
 * @file vision_node.cpp
 * @brief The main file for the vision node
 */

#include <ros/ros.h>
#include "vision.h"
#include "constants.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, VISION_NODE);

    Vision v;

    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}