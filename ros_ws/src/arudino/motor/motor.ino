/**
 * @file motor.ino
 * @brief Controls the Motors
 * 
 * Code to be put on the Arduino based Motor Controllers
 * Converts Angular Velocity (Rad/Sec) to intiger (0-255)
 * 
 */

#include <ros.h>
#include <std_msgs/Float64.h>
//#include <common/constants.h>

/*************
 * Constants *
 *************/
 
/**
 * @brief Define Pi for use in conversion
 */
const float PI_CONST = 3.14159265359;
 
/**
 * @brief Radians Per Second at full Throtle
 */
const float MAX_RAD_SEC = (8*PI_CONST);

/**
 * @brief Pins used to control the Motor
 */
const int FORWARD_PWM_PIN = 3;
const int REVERSE_PWM_PIN = 4;
const int ENABLE_PIN = 5;
 
/************************
 * Forward Declerations *
 ************************/
void velocityCallback(const std_msgs::Float64& msg);

/**
 * @brief ROS node handle
 */
ros::NodeHandle nodeHandle;

/**
 * @brief ROS Subscriber for the Velocity Message
 */
ros::Subscriber<std_msgs::Float64> velocitySub("wheel_velocity", &velocityCallback);

/**
 * @brief The callback function for velocity messages
 * 
 * This function accepts velocity messages and convertes them into wheel speeds
 * This will convert a float into an intger(0-255)
 * 
 * @param msg The message witch is recived from the velocity publisher
 */
void velocityCallback(const std_msgs::Float64& msg)
{
    int speed = map(abs(msg.data), 0, MAX_RAD_SEC, 0, 255);
    if(msg.data > 0)
    {
        //Go Forward
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, speed);
        analogWrite(REVERSE_PWM_PIN, 0);
    }
    else if(msg.data < 0)
    {
        //Go in Reverse
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, 0);
        analogWrite(REVERSE_PWM_PIN, speed);
    }
    else
    {
        //active break
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, 0);
        analogWrite(REVERSE_PWM_PIN, 0);
    }
}

void setup() 
{
    //setup pins
    pinMode(FORWARD_PWM_PIN, OUTPUT);
    pinMode(REVERSE_PWM_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    //Set Robot to break when starting
    digitalWrite(ENABLE_PIN, HIGH);
    analogWrite(FORWARD_PWM_PIN, 0);
    analogWrite(REVERSE_PWM_PIN, 0);

    //Setup ROS node and topics
    nodeHandle.initNode();
    nodeHandle.subscribe(velocitySub);
}

void loop() 
{
    nodeHandle.spinOnce();
}
