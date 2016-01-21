/**
 * @file constants.h
 * @brief A file containing constants used throughout all of the IGVC packages
 */

#ifndef COMMON_CONSTANTS_H_
#define COMMON_CONSTANTS_H_

#include "config.h"

//Topic name constants
/**
 * @brief The topic that joy messages will be received from
 */
const std::string JOY_TOPIC = "joy";
/**
 * @brief The topic that teleop twist messages will be published on
 */
const std::string TELEOP_TOPIC = "teleop_twist";
/**
 * @brief The topic that autonomous twist messages will be published on
 */
const std::string AUTO_TOPIC = "auto_twist";
/**
 * @brief The topic containing twist messages to be sent to the motor controller
 */
const std::string CONTROL_TOPIC = "cmd_vel";
/**
 * @brief The topic that the right wheel angular velocity will be published on
 */
const std::string RIGHT_VEL_TOPIC = "right_vel";
/**
 * @brief The topic that the left wheel angular velocity will be published on
 */
const std::string LEFT_VEL_TOPIC = "left_vel";
/**
 * @brief The topic that the left encoder will publish on
 */
const std::string LEFT_ENCODER_TOPIC = "left_tick";
/**
 * @brief The topic that the right encoder will publish on
 */
const std::string RIGHT_ENCODER_TOPIC = "right_tick";

//Node name constants
/**
 * @brief The node name for the teleop node
 */
const std::string TELEOP_NODE = "teleop_control";
/**
 * @brief The node name for the teleop_mux node
 */
const std::string TELEOP_MUX_NODE = "teleop_mux_node";
/**
 * @brief The node name for the motor_controller node
 */
const std::string MOTOR_CONTROLLER_NODE = "motor_controller_node";

//Loop rate
/**
 * @brief The rate that the loops will run at in Hz
 */
const int LOOP_RATE = 30;

#endif /* COMMON_CONSTANTS_H_ */
