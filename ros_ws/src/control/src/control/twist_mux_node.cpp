/**
 * @file teleop_mux_node.cpp
 * @brief The main file for the teleop_mux node
 */

#include <ros/ros.h>
#include "twist_mux.h"
#include "constants.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, TELEOP_MUX_NODE);

    TwistMux tw;

    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok())
    {
        tw.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
