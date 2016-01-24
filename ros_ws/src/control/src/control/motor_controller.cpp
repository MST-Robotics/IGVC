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
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("wheel_states", 1);

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

    //Setup odom tf link
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    prev_theta = 0;

    //Setup the joint state message
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.name[0] = "left_wheel_joint";
    joint_state.name[1] = "right_wheel_joint";
    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    
    unscaled_max_speed = (2 * max_lin_vel + max_ang_vel * robot_base) / (2 * wheel_rad);
}

void MotorController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    right_vel.data = getRightVel(msg->linear.x, msg->angular.z);
    left_vel.data = getLeftVel(msg->linear.x, msg->angular.z);
}

void MotorController::leftEncoderCallback(const control::Encoder::ConstPtr& msg)
{
    displacement_left = ((M_PI * robot_base) / encoder_res) * msg->ticks;

    //Calculate rotational position of the wheel
    joint_state.position[0] += ((2 * M_PI / encoder_res) * msg->ticks)  - M_PI;

    if(joint_state.position[0] > M_PI)
        joint_state.position[0] -= 2*M_PI;
    else if(joint_state.position[0] < -M_PI)
        joint_state.position[0] += 2*M_PI;
}

void MotorController::rightEncoderCallback(const control::Encoder::ConstPtr& msg)
{
    displacement_right = ((M_PI * robot_base) / encoder_res) * msg->ticks;

            //Calculate rotational position of the wheel
    joint_state.position[1] += ((2 * M_PI / encoder_res) * msg->ticks)  - M_PI;

    if(joint_state.position[1] > M_PI)
        joint_state.position[1] -= 2*M_PI;
    else if(joint_state.position[1] < -M_PI)
        joint_state.position[1] += 2*M_PI;
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
    joint_state.header.stamp = ros::Time::now();
    joint_state_pub.publish(joint_state);

    odom_trans.header.stamp = ros::Time::now();

    double displacement = (displacement_right + displacement_left) / 2;
    double theta = (displacement_right - displacement_left) / robot_base;

    odom_trans.transform.translation.x += displacement * cos(theta);
    odom_trans.transform.translation.y += displacement * sin(theta);
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(prev_theta + theta);

    odom_broadcaster.sendTransform(odom_trans);

    prev_theta += theta;

    right_vel_pub.publish(right_vel);
    left_vel_pub.publish(left_vel);
}
