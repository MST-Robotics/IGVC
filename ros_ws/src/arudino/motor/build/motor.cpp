#line 1 "motor.ino"
   
                  
                             
   
                                                        
                                                         
   
   

#include <ros.h>
#include <std_msgs/Float64.h>
                               

              
              
               
 
   
                                         
   
#include "Arduino.h"
void setup();
void loop();
#line 21
const float PI_CONST = 3.14159265359;
 
   
                                            
   
const float MAX_RAD_SEC = (8*PI_CONST);

   
                                        
   
const int FORWARD_PWM_PIN = 5;
const int REVERSE_PWM_PIN = 6;
const int ENABLE_PIN = A3;

#define RIGHT_WHEEL

#if defined RIGHT_WHEEL
const char* VELOCITY_TOPIC = "right_vel";
#elif defined LEFT_WHEEL
const char* VELOCITY_TOPIC = "left_vel";
#endif

                         
                         
                          
void velocityCallback(const std_msgs::Float64& msg);

   
                         
   
ros::NodeHandle nodeHandle;

   
                                                 
   
ros::Subscriber<std_msgs::Float64> velocitySub(VELOCITY_TOPIC, &velocityCallback);

   
                                                     
   
                                                                               
                                                  
   
                                                                      
   
void velocityCallback(const std_msgs::Float64& msg)
{
    int speed = map(abs(msg.data), 0, MAX_RAD_SEC, 0, 255);
    if(msg.data > 0)
    {
                    
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, speed);
        analogWrite(REVERSE_PWM_PIN, 0);
    }
    else if(msg.data < 0)
    {
                       
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, 0);
        analogWrite(REVERSE_PWM_PIN, speed);
    }
    else
    {
                      
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, 0);
        analogWrite(REVERSE_PWM_PIN, 0);
    }
}

void setup() 
{
                
    pinMode(FORWARD_PWM_PIN, OUTPUT);
    pinMode(REVERSE_PWM_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

                                      
    digitalWrite(ENABLE_PIN, HIGH);
    analogWrite(FORWARD_PWM_PIN, 0);
    analogWrite(REVERSE_PWM_PIN, 0);

                               
    nodeHandle.initNode();
    nodeHandle.subscribe(velocitySub);
}

void loop() 
{
    nodeHandle.spinOnce();
}

