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
    left_encoder = nh.subscribe<control::Encoder>(LEFT_ENCODER_TOPIC, 5, &MotorController::leftEncoderCallback, this);
    right_encoder = nh.subscribe<control::Encoder>(RIGHT_ENCODER_TOPIC, 5, &MotorController::rightEncoderCallback,
                                                   this);

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
    if(!nh.getParam("/encoder_res", encoder_res))
    {
        ROS_ERROR("encoder_res is not defined on the parameter server");
    }
    
    unscaled_max_speed = (2 * max_lin_vel + max_ang_vel * robot_base) / (2 * wheel_rad);
}

void MotorController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    right_vel.data = getRightVel(msg->linear.x, msg->angular.z);
    left_vel.data = getLeftVel(msg->linear.x, msg->angular.z);
}

void MotorController::rightEncoderCallback(const control::Encoder::ConstPtr& msg)
{
    double dt = (msg->header.stamp.toSec() - right_prev_time.toSec());

    //Calculate wheel velocity in m/s
    right_measured_dist = (static_cast<double>(msg->ticks) * wheel_rad * 2 * M_PI)/encoder_res;

    //Set the previous time for next calculation
    right_prev_time = msg->header.stamp;
}

void MotorController::leftEncoderCallback(const control::Encoder::ConstPtr& msg)
{
    double dt = (msg->header.stamp.toSec() - right_prev_time.toSec());

    //Calculate wheel velocity in m/s
    left_measured_dist = (static_cast<double>(msg->ticks) * wheel_rad * 2 * M_PI)/encoder_res;

    left_prev_time = msg->header.stamp;
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

double MotorController::scale(const double val, const double pre_min, const double pre_max, const double scale_min,
                              const double scale_max)
{
    return (((scale_max - scale_min)*(val - pre_min))/(pre_max - pre_min)) + scale_min;
}

void MotorController::update()
{
    //Calculate robot linear and angular velocities
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion rot;
    double trans, theta;

    theta = (right_measured_dist - left_measured_dist) / robot_base;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(theta), rot);
    trans = (right_measured_dist + left_measured_dist) / 2;

    //Build odometry header message
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.child_frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = trans * cos(theta);
    odom_trans.transform.translation.y = trans * sin(theta);
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = rot;

    odom_broadcaster.sendTransform(odom_trans);

    right_vel_pub.publish(right_vel);
    left_vel_pub.publish(left_vel);
}
