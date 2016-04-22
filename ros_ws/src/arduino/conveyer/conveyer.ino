/**
 * @file conveyor.ino
 * @brief Controls the conveyor Belt
 * 
 * Code to be put on the Arduino based Motor Controllers
 * Converts Angular Velocity (Rad/Sec) to float (0-255)
 * 
 */

#include <ros.h>
#include <std_msgs/Float64.h>

const char* conveyor_TOPIC = "conveyor_vel";

/**
 * @brief Pins used to control the Motor
 */
const int FORWARD_PWM_PIN = 10;
const int REVERSE_PWM_PIN = 9;
const int ENABLE_PIN = 7;

void conveyorCallback(const std_msgs::Float64& msg);
float fScale( float originalMin, float originalMax, float newBegin, 
              float newEnd, float inputValue, float curve);

/**
 * @brief Float used to scale the Speed to
 */
float desired_speed = 0;

bool flag = false;

/**
 * @brief Boolean used to store the desired direction
 * 
 * True is Forward, and False is Backwards
 */
float desired_direction;

/**
 * @brief ROS node handle
 */
ros::NodeHandle node_handle;

/**
 * @brief ROS Subscriber for the Velocity Message
 */
ros::Subscriber<std_msgs::Float64> conveyorSub(conveyor_TOPIC, &conveyorCallback);

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
    node_handle.subscribe(conveyorSub);
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
void conveyorCallback(const std_msgs::Float64& msg)
{
    if(!flag && msg.data == 0)
    {
        flag = true;
    }
  
    desired_speed = fScale(0, 1, 0, 255, abs(msg.data), 0);
    //If msg.data is positive, the set to TRUE
    desired_direction = (msg.data > 0);
    
    if(msg.data > 0 && flag)
    {
        //Go Forward
        digitalWrite(ENABLE_PIN, HIGH);
        analogWrite(FORWARD_PWM_PIN, desired_speed);
        analogWrite(REVERSE_PWM_PIN, 0);
    }
    else if(msg.data < 0 && flag)
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

