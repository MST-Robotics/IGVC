/**
 * @file motor_controller_node.cpp
 * @brief The motor controller node source file
 */

#include <ros/ros.h>
#include "motor_controller.h"
#include "constants.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, MOTOR_CONTROLLER_NODE);

    MotorController mc;

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok())
    {
        mc.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}




