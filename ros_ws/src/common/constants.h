/**
 * @file constants.h
 * @brief A file containing constants used throughout all of the IGVC packages
 */

#ifndef COMMON_CONSTANTS_H_
#define COMMON_CONSTANTS_H_

//Controller Constants
/**
 * @brief The index of the y axis on the left stick of the controller
 */
const int Y_AXIS_L_STICK = 1;
/**
 * @brief The index of the x axis on the right stick of the controller
 */
const int X_AXIS_R_STICK = 2;

//Topic name constants
/**
 * @brief The topic that joy messages will be received from
 */
const std::string JOY_TOPIC = "joy";
/**
 * @brief The topic that twist messages will be published on
 */
const std::string TELEOP_TOPIC = "teleop_twist";

//Node name constants
/**
 * @brief The node name for the teleop node
 */
const std::string TELEOP_NODE = "teleop_control";

//Loop rate
/**
 * @brief The rate that the loops will run at in Hz
 */
const int LOOP_RATE = 30;

#endif /* COMMON_CONSTANTS_H_ */
