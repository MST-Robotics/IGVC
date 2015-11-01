#include "config.h"




ConfigValues initConfigs()
{
    ros::NodeHandle nh;
    ConfigValues config;
    if(!nh.getParam("/robot_base", config.robot_base))
    {
    	ROS_ERROR("robot_base is not defined on parameter server");
    }
    if(!nh.getParam("/wheel_rad", config.wheel_rad))
    {
    	ROS_ERROR("wheel_rad is not defined on the parameter server");
    }
    if(!nh.getParam("/max_speed", config.max_speed))
    {
    	ROS_ERROR("max_speed is not defined on the parameter server");
    }
    if(!nh.getParam("/max_lin_vel", config.max_lin_vel))
    {
    	ROS_ERROR("max_lin_vel is not defined on the parameter server");
    }
    if(!nh.getParam("/max_ang_vel", config.max_ang_vel))
    {
    	ROS_ERROR("max_ang_vel is not defined on the parameter server");
    }
    
    if(!nh.getParam("/controller_type", config.controller_type))
    {
    	ROS_ERROR("controller_type is not defined on parameter server");
    } 
    if(!nh.getParam("/lin_vel_btn", config.lin_vel_btn))
    {
    	ROS_ERROR("lin_vel_btn is not defined on the parameter server");
    }
    if(!nh.getParam("/ang_vel_btn", config.ang_vel_btn))
    {
    	ROS_ERROR("ang_vel_btn is not defined on the parameter server");
    }
    if(!nh.getParam("/teleop_btn", config.teleop_btn))
    {
    	ROS_ERROR("teleop_btn is not defined on the parameter server");
    }
    if(!nh.getParam("/standby_btn", config.standby_btn))
    {
    	ROS_ERROR("standby_btn is not defined on the parameter server");
    }
    if(!nh.getParam("/autonomous_btn", config.autonomous_btn))
    {
    	ROS_ERROR("autonomous_btn is not defined on the parameter server");
    }
    if(!nh.getParam("/speed_inc_btn", config.speed_inc_btn))
    {
    	ROS_ERROR("speed_inc_btn is not defined on the parameter server");
    }
    if(!nh.getParam("/speed_dec_btn", config.speed_dec_btn))
    {
    	ROS_ERROR("speed_dec_btn is not defined on the parameter server");
    }
    return config;
}


