/**
 * @file vision_node.cpp
 * @brief The vision node source file
 */

#include "vision.h"

int main(int argc, char** argv)
{
    //Setup ROS
    ros::init(argc, argv, "vision_node");

    Vision v;

    //Set the rate to 30 Hertz
    ros::Rate loop_rate(30);

    //While ROS is running
    while(ros::ok())
    {
        //Check the Callbacks
        ros::spinOnce();
        loop_rate.sleep();   
        //Update ROS with the processed message
        v.update(); 
    }
}
