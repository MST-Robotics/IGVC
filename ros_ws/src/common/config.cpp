/**
 * @file config.cpp
 * @brief ConfigValues functions
 * These functions are used with the config values struct.
 */

#include "config.h"
/**
 * @brief Gets an integer parameter.
 *
 * Due to an issue with getting an int param directly, this function was made
 *
 * @param key_name The name of the config option
 * @param config_var The variable to store the retrived value in
 */
void get_int_param(std::string key_name, int& config_var)
{
    ros::NodeHandle nh; 
    double casted = static_cast<double>(config_var);
    if(!nh.getParam(key_name, casted))
    {
    	ROS_ERROR("%s is not defined on the parameter server", key_name.c_str());
    }
    config_var = static_cast<int>(casted);
    return;
}

/**
 * @brief Initilizes the configs.
 *
 * Loads the config values from the parameter server.
 *
 */
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
    //controller mapping
    get_int_param("/lin_vel_btn", config.lin_vel_btn);
    get_int_param("/ang_vel_btn", config.ang_vel_btn);
    get_int_param("/teleop_btn", config.teleop_btn);
    get_int_param("/standby_btn", config.standby_btn);
    get_int_param("/autonomous_btn", config.autonomous_btn);
    get_int_param("/speed_inc_btn", config.speed_inc_btn);
    get_int_param("/speed_dec_btn", config.speed_dec_btn);
    return config;
}

