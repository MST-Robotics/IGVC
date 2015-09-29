/**
 * @file remote_control_node.cpp
 * @brief The main file for the remote control node
 */

#include <ros/ros.h>
#include "remote_control.h"
#include "constants.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, TELEOP_NODE);

    RemoteControl con;

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok())
    {
        con.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

