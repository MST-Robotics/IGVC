/**
 * @file actuator.ino
 * @brief Controls the Dump
 * 
 * Code to be put on the Arduino based Motor Controllers
 * Turns Linear Actuators On and Off
 * 
 */

 #include <ros.h>
#include <std_msgs/Int8.h>

const char* DUMP_TOPIC = "dump_activate";

/**
 * @brief Pins used to control the Motor
 */
const int FORWARD_PWM_PIN = 10;
const int REVERSE_PWM_PIN = 9;
const int ENABLE_PIN = 7;

void dumpCallback(const std_msgs::Int8& msg);

/**
 * @brief ROS node handle
 */
ros::NodeHandle node_handle;

/**
 * @brief ROS Subscriber for the Velocity Message
 */
ros::Subscriber<std_msgs::Int8> dumpSub(DUMP_TOPIC, &dumpCallback);

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
    node_handle.initNode();
    node_handle.subscribe(dumpSub);
}

void loop() 
{
    //Only used to get the messages from ROS
    node_handle.spinOnce();
}

/**
 * @brief The callback function for velocity messages
 * 
 * This function accepts velocity messages and convertes them into wheel speeds
 * This will convert a float into an intger(0-255)
 * 
 * @param msg The message witch is recived from the velocity publisher
 */
void dumpCallback(const std_msgs::Int8& msg)
{
    if(msg.data == 1)
    {
        //Go Forward
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, 255);
        analogWrite(REVERSE_PWM_PIN, 0);
    }
    else if(msg.data == -1)
    {
        //Go in Reverse
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, 0);
        analogWrite(REVERSE_PWM_PIN, 255);
    }
    else
    {
        //active break
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, 0);
        analogWrite(REVERSE_PWM_PIN, 0);
    }
}

