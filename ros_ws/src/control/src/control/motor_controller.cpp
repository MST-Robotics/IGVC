/**
 * @file motor_controller.cpp
 * @brief The implementation file for the MotorController class
 */

#include <motor_controller.h>

MotorController::MotorController()
{
    //Initialize publishers and subscribers
    twist_sub = nh.subscribe<geometry_msgs::Twist>(CONTROL_TOPIC, 1, &MotorController::twistCallback, this);
    right_vel_pub = nh.advertise<std_msgs::Float64>(RIGHT_VEL_TOPIC, 1);
    left_vel_pub = nh.advertise<std_msgs::Float64>(LEFT_VEL_TOPIC, 1);
}

void MotorController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    right_vel.data = getRightVel(msg->linear.x, msg->angular.z);
    left_vel.data = getLeftVel(msg->linear.x, msg->angular.z);
}

double MotorController::getRightVel(const double lin_vel, const double ang_vel)
{
    return (2 * lin_vel + ang_vel * ROBOT_BASE) / (2 * WHEEL_RAD);
}

double MotorController::getLeftVel(const double lin_vel, const double ang_vel)
{
    return (2 * lin_vel - ang_vel * ROBOT_BASE) / (2 * WHEEL_RAD);
}

void MotorController::update()
{
    right_vel_pub.publish(right_vel);
    left_vel_pub.publish(left_vel);
}
