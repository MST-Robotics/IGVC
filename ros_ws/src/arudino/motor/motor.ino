/**
 * @file motor.ino
 * @brief Controls the Motors
 */

#include <ros.h>
#include <std_msgs/Float64.h>

#define pi 3.14

/*************
 * Constants *
 *************/
 //Radians Per Second at full Throtall
const float MAX_RAD_SEC = (8*pi);

//Pins used to controll the Motor
const int FORWARD_PWM_PIN = 3;
const int REVERSE_PWM_PIN = 4;
const int ENABLE_PIN = 5;
 
/************************
 * Forward Declerations *
 ************************/
void VelocityCallback(const std_msgs::Float64 &msg);

//ROS node handle
ros::NodeHandle nodeHandle;

//ROS Subscriber for the Velocity Message
ros::Subscriber<std_msgs::Float64> 
     velocitySub("wheel_velocity", &VelocityCallback);

/*************
 * Callbacks *
 *************/
void VelocityCallback(const std_msgs::Float64 &msg)
{
  int speed;
  if(msg.data > 0)
  {
    speed = map(msg.data, 0, MAX_RAD_SEC, 0, 255);
    //Go Forward
    digitalWrite(ENABLE_PIN, HIGH);
    analogWrite(FORWARD_PWM_PIN, speed);
    analogWrite(REVERSE_PWM_PIN, 0);
  }
  else if(msg.data < 0)
  {
    speed = map(abs(msg.data), 0, MAX_RAD_SEC, 0, 255);
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

void setup() {
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

void loop() {
  nodeHandle.spinOnce();
}
