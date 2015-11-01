/**
 * @file config.h
 * @brief A file containing the various things involving config files
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
double lin_vel_btn;
double ang_vel_btn;
double teleop_btn;
double standby_btn;
double autonomous_btn;
double speed_inc_btn;
double speed_dec_btn;
double turbo_btn;


};
ConfigValues initConfigs();

 
#endif /* CONFIG_H_ */
