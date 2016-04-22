/**
 * @file motor.ino
 * @brief Controls the Motors
 * 
 * Code to be put on the Arduino based Motor Controllers
 * Converts Angular Velocity (Rad/Sec) to float (0-255)
 * 
 */

#include <ros.h>
#include <std_msgs/Float64.h>
#include <control/Encoder.h>
#include <ros/time.h>

/*************
 * Constants *
 *************/
 
/**
 * @brief Place to Change which Wheel we want to Program
 * 
 * In order to correctly inturpret the Wheel Velocity's, 
 *  we need to define which wheel we are uploading code to. 
 *  This will be used to insure that the proper wheel gets the
 *  Proper commands
 */
#define R_WHEEL

#if defined R_WHEEL
const char* VELOCITY_TOPIC = "right_vel";
const char* ENCODER_TOPIC = "right_tick";
#elif defined L_WHEEL
const char* VELOCITY_TOPIC = "left_vel";
const char* ENCODER_TOPIC = "left_tick";
#endif

/**
 * @brief value for the desierd refresh rate of the encoder
 * 
 * Variable is the number of Millisecond Delay between publishes
 */
 const float REFRESH_RATE_DESIRED = 1000;
 
/******************************************************************
 * Adjust for the changing of the arudino timer funciton to       *
 * remove motor whine. This will only work with a divisor of 8!   *
 * If Divisor changes, please look into how this effects arduino  *
 * Timing Functons.                                               *
 *                                                                *
 * For Referance please see below website                         *
 * http://playground.arduino.cc/Main/TimerPWMCheatsheet           *
 * ****************************************************************/
 //const float REFRESH_RATE = (REFRESH_RATE_DESIRED * 8);

 //Replace this line with the line above it if motor whine 
 //   is still an issue
   const float REFRESH_RATE = (REFRESH_RATE_DESIRED);
   
/**
 * @brief Radians Per Second at full Throtle
 * 
 * todo !!!!!!!STILL NEEDS TO BE UPDATED!!!!!!!!!
 * 
 */
const float MAX_RAD_SEC = 5;

/**
 * @brief Pin the encoder is attached
 */
const int ENCODER_PIN = 2;

/**
 * @brief Pins used to control the Motor
 */
const int FORWARD_PWM_PIN = 5;
const int REVERSE_PWM_PIN = 6;
const int ENABLE_PIN = A3;

/**
 * @brief Float used to scale the Speed to
 */
float desired_speed = 0;

/**
 * @brief Boolean used to store the desired direction
 * 
 * True is Forward, and False is Backwards
 */
float desired_direction;

/**
 * @brief Values to be updated when the inturrupt is triggered for encoder
 */
volatile unsigned int encoder_ticks = 0;

/**
 * @brief Values used to keep track of current time for multitasking
 */
 int current_mills = 0;
 int old_mills = 0;

/************************
 * Forward Declerations *
 ************************/
void encoderCount();
void updateEncoder();
void set_pwm_frequency();
void velocityCallback(const std_msgs::Float64& msg);
float fScale( float originalMin, float originalMax, float newBegin, 
              float newEnd, float inputValue, float curve);

/**
 * @brief ROS node handle
 */
ros::NodeHandle node_handle;

/**
 * @brief ROS message used for publishing the encoder data
 */
control::Encoder encoderMessage;

/**
 * @brief ROS Subscriber for the Velocity Message
 */
ros::Subscriber<std_msgs::Float64> velocitySub(VELOCITY_TOPIC, &velocityCallback);

/**
 * @brief ROS Publisher for the Encoder Message
 */
ros::Publisher encoderPub(ENCODER_TOPIC, &encoderMessage);

void setup() 
{
    //Fix the Motor Whine
    //set_pwm_frequency();

    //setup pins
    pinMode(FORWARD_PWM_PIN, OUTPUT);
    pinMode(REVERSE_PWM_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(ENCODER_PIN, INPUT);

    //Set Robot to break when starting
    digitalWrite(ENABLE_PIN, HIGH);
    analogWrite(FORWARD_PWM_PIN, 0);
    analogWrite(REVERSE_PWM_PIN, 0);

    //Setup ROS node and topics
    node_handle.initNode();
    node_handle.subscribe(velocitySub);
    node_handle.advertise(encoderPub);

    //Set the Inturupt on Pin 2
    attachInterrupt(0, encoderCount, RISING);
}

void loop() 
{
    //update the current time for multitasking
    current_mills = millis();
    
    //Only update the Encoder data when the Refresh Rate says to
    if(current_mills-old_mills >= REFRESH_RATE)
    {
        updateEncoder();
        old_mills = current_mills;
    }

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
void velocityCallback(const std_msgs::Float64& msg)
{
    desired_speed = fScale(0, MAX_RAD_SEC, 0, 255, abs(msg.data), 0);
    //If msg.data is positive, the set to TRUE
    desired_direction = (msg.data > 0);
    
    if(msg.data > 0)
    {
        //Go Forward
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, desired_speed);
        analogWrite(REVERSE_PWM_PIN, 0);
    }
    else if(msg.data < 0)
    {
        //Go in Reverse
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, 0);
        analogWrite(REVERSE_PWM_PIN, desired_speed);
    }
    else
    {
        //active break
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, 0);
        analogWrite(REVERSE_PWM_PIN, 0);
    }
}

/**
 * @brief The Function will send the updated encoder value to ROS
 * 
 * This Function is used to update the main ROS program with
 * the current number of ticks per timestamp
 */
void updateEncoder()
{
    //update the value of the message
    encoderMessage.ticks = encoder_ticks;
    encoderMessage.header.stamp = node_handle.now();
    
    //publish message
    encoderPub.publish(&encoderMessage);

    //reset the count to 0
    encoder_ticks = 0;
}

/**
 * @brief The Function will update the encoder
 * 
 * This Function is called whenever there is a change 
 * to the value of pin 2, it is part of the attachInterrupt routine
 * It updates the value of encoder_ticks
 */
void encoderCount()
{
    if(desired_direction)
    {
        encoder_ticks++; 
    }
    else
    {
        encoder_ticks--;
    }
}

/**
 * @brief The Function will Scale the Input to the Expected Output
 * 
 * This function accepts the input value, and input range,
 * It will the scale the value to the expected output inside
 * the given output range
 * This Function will also allow the input to be scaled
 * with a curve, It is not onyl linear.
 *  
 * @param originalMin - minimum input expected by function
 *        originalMax - maximum input expected by function
 *        newBegin - minimum input returned by the function
 *        newEnd - maximum input returned by the function
 *        inputValue - the value you want scaled
 *        curve - excepts a value -10 through 10, 0 being linear
 *                scaling the values with a curve
 */
float fScale( float original_min, float original_max, float new_begin, 
              float new_end, float input_value, float curve)
{

    float original_range = 0;
    float new_range = 0;
    float zero_ref_cur_val = 0;
    float normalized_cur_val = 0;
    float ranged_value = 0;
    boolean inv_flag = 0;


    // condition curve parameter
    // limit range
    if (curve > 10) 
    {
        curve = 10;
    }
    if (curve < -10) 
    {
        curve = -10;
    }

    // - invert and scale - this seems more intuitive - 
    //   postive numbers give more weight to high end on output
    curve = (curve * -.1) ; 
    // convert linear scale into lograthimic exponent for other pow function
    curve = pow(10, curve);

    // Check for out of range inputValues
    if (input_value < original_min)
    {
        input_value = original_min;
    }
    if (input_value > original_max) 
    {
        input_value = original_max;
    }

    // Zero Refference the values
    original_range = original_max - original_min;

    if (new_end > new_begin)
    {
        new_range = new_end - new_begin;
    }
    else
    {
        new_range = new_begin - new_end;
        inv_flag = 1;
    }

    zero_ref_cur_val = input_value - original_min;
    
    // normalize to 0 - 1 float
    normalized_cur_val  =  zero_ref_cur_val / original_range;

    // Check for originalMin > originalMax  - the math for all other cases 
    //   i.e. negative numbers seems to work out fine
    if (original_min > original_max ) 
    {
        return 0;
    }

    if (inv_flag == 0)
    {
        ranged_value =  (pow(normalized_cur_val, curve) * new_range) + new_begin;
    }
    else     // invert the ranges
    {  
      ranged_value =  new_begin - (pow(normalized_cur_val, curve) * new_range);
    }

    return ranged_value;
}

/**
 * @brief The Function will change frequency of Timer 0
 * 
 * This function will change the frequency of the Timer 
 * that controlls the PWM signal. This will alos effect 
 * any other functions on Timer 0
 */
void set_pwm_frequency() 
{

    //set mode of timer 0
    TCCR0B = TCCR0B & 0b11111000 | 0x02;

    return;
}
