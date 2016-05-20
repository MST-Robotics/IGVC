/**
 * @file light.ino
 * @brief Controls the Status Light
 * 
 * Code to be put on the Arduino based Status Light Board
 * Will differentiate between Autonomus and Tele-Op
 * 
 */

#include <ros.h>
#include <std_msgs/Bool.h>

/*************
 * Constants *
 *************/
/**
 * @brief Pin the Status Light is attached
 * 
 * If this pin gets set high, then the light is flashing signifying Autonomus Mode
 */
#define LIGHT_PIN 7


const char* LIGHT_TOPIC = "light_status";

/************************
 * Forward Declerations *
 ************************/
 
void lightCallback(const std_msgs::Bool& msg);


/**
 * @brief ROS node handle
 */
ros::NodeHandle node_handle;

/**
 * @brief ROS Subscriber for the Light Message
 */
ros::Subscriber<std_msgs::Bool> lightSub(LIGHT_TOPIC, &lightCallback);

void setup()
{
    //setup pins
    pinMode(LIGHT_PIN, OUTPUT);

    digitalWrite(LIGHT_PIN, LOW);

    //Setup ROS node and topics
    node_handle.initNode();
    node_handle.subscribe(lightSub);    
  
}

void loop()
{
    //Only used to get the messages from ROS
    node_handle.spinOnce();
}

void lightCallback(const std_msgs::Bool& msg)
{
    if(msg.data)
    {
        digitalWrite(LIGHT_PIN, HIGH);
    }
    else
    {
        digitalWrite(LIGHT_PIN, LOW);
    }
}

