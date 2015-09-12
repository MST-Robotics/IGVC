/**
 * @file remote_control_node.cpp
 * @brief The main file for the remote control node
 */

#include "remote_control.h"

const std::string G_TELEOP_TOPIC = "teleop_control";

int main(int argc, char** argv)
{
    ros::init(argc, argv, G_TELEOP_TOPIC);

    RemoteControl con;

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        con.update();
        ros::spinOnce();
    }
}

