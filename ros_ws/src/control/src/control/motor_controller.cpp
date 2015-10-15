/**
 * @file motor_controller.cpp
 * @brief The implementation file for the MotorController class
 */

#include <motor_controller.h>

MotorController::MotorController()
{
	double max_lin_vel, max_ang_vel;

    //Initialize publishers and subscribers
    twist_sub = nh.subscribe<geometry_msgs::Twist>(CONTROL_TOPIC, 1, &MotorController::twistCallback, this);
    right_vel_pub = nh.advertise<std_msgs::Float64>(RIGHT_VEL_TOPIC, 1);
    left_vel_pub = nh.advertise<std_msgs::Float64>(LEFT_VEL_TOPIC, 1);

    //Check that the necessary parameters exist and set member variables appropriately
    if(!nh.getParam("/robot_base", robot_base))
    {
    	ROS_ERROR("robot_base is not defined on parameter server");
    }
    if(!nh.getParam("/wheel_rad", wheel_rad))
    {
    	ROS_ERROR("wheel_rad is not defined on the parameter server");
    }
    if(!nh.getParam("/max_speed", max_speed))
    {
    	ROS_ERROR("max_speed is not defined on the parameter server");
    }
    if(!nh.getParam("/max_lin_vel", max_lin_vel))
    {
    	ROS_ERROR("max_lin_vel is not defined on the parameter server");
    }
    if(!nh.getParam("/max_ang_vel", max_ang_vel))
    {
    	ROS_ERROR("max_ang_vel is not defined on the parameter server");
    }

    unscaled_max_speed = (2 * max_lin_vel + max_ang_vel * robot_base) / (2 * wheel_rad);
}

void MotorController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    right_vel.data = getRightVel(msg->linear.x, msg->angular.z);
    left_vel.data = getLeftVel(msg->linear.x, msg->angular.z);
}

double MotorController::getRightVel(const double lin_vel, const double ang_vel)
{
	double pre_scaled = (2 * lin_vel + ang_vel * robot_base) / (2 * wheel_rad);
    return scale(pre_scaled, 0, unscaled_max_speed, 0, max_speed);
}

double MotorController::getLeftVel(const double lin_vel, const double ang_vel)
{
	double pre_scaled = (2 * lin_vel - ang_vel * robot_base) / (2 * wheel_rad);
	return scale(pre_scaled, 0, unscaled_max_speed, 0, max_speed);
}

double MotorController::scale(const double val, const double pre_min, const double pre_max, const double scale_min, const double scale_max)
{
	return (((scale_max - scale_min)*(val - pre_min))/(pre_max - pre_min)) + scale_min;
}

void MotorController::update()
{
    right_vel_pub.publish(right_vel);
    left_vel_pub.publish(left_vel);
}
