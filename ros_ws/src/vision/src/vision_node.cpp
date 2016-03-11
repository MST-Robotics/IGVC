/**
 * @file vision_node.cpp
 * @brief The vision node source file
 */

#include "vision.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");

    Vision v;

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        v.update();    
    }
}
