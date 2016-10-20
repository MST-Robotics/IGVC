/**
 * @file config.h
 * @struct ConfigValues
 * @brief Contains the struct ConfigValues.
 *
 * To use ConfigValues to access a config value:
 * add "ConfigValues config;" to the class constructor and
 * add "config = initConfigs()" where ever you need to use it
 * @see initConfigs()
 */

#ifndef CONFIG_H_
#define CONFIG_H_


#include <ros/ros.h>
#include "constants.h"


struct ConfigValues
{
double robot_base;
double wheel_rad;
double max_speed;
double max_lin_vel;
double max_ang_vel;

//Controller Info
std::string controller_type;
int left_y_axis;
int right_x_axis;
int right_x_axis;
int teleop_btn;
int standby_btn;
int autonomous_btn;
int speed_inc_btn;
int speed_dec_btn;
int turbo_btn;


};
ConfigValues initConfigs();
void get_int_param(std::string key_name, int & config_var);

 
#endif /* CONFIG_H_ */
