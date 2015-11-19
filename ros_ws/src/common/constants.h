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
/**
 * @brief The index of the A button on the controller
 */
const int A_BUTTON = 0;
/**
 * @brief The index of the B button on the controller
 */
const int B_BUTTON = 1;
/**
 * @brief The index of the X button on the controller
 */
const int X_BUTTON = 2;
/**
 * @brief The index of the Y button on the controller
 */
const int Y_BUTTON = 3;
/**
 * @brief The index of the LB button on the controller
 */
const int LB_BUTTON = 4;
/**
 * @brief The index of the RB button on the controller
 */
const int RB_BUTTON = 5;

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
 * @brief The topic that images for processing will be received from
 */
const std::string VISION_INCOMING_TOPIC = "usb_cam/image_raw";
/**
 * @brief The topic that images are published to after being processed
 */
const std::string VISION_OUTGOING_TOPIC = "processed_image";

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
/**
 * @brief THe node name for the vision node
 */
const std::string VISION_NODE = "vision_node";

//Loop rate
/**
 * @brief The rate that the loops will run at in Hz
 */
const int LOOP_RATE = 30;

#endif /* COMMON_CONSTANTS_H_ */
