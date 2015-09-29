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
const int FORWARD_PWM_PIN = 5;
const int REVERSE_PWM_PIN = 6;
const int ENABLE_PIN = A3;


/**
 * @brief Place to Change which Wheel we want to Program
 * 
 * In order to correctly inturpret the Wheel Velocity's, 
 *  we need to define which wheel we are uploading code to. 
 *  This will be used to insure that the proper wheel gets the
 *  Proper commands
 */
#define RIGHT_WHEEL

#if defined RIGHT_WHEEL
const char* VELOCITY_TOPIC = "right_vel";
#elif defined LEFT_WHEEL
const char* VELOCITY_TOPIC = "left_vel";
#endif

/**
 * @brief Float used to scale the Speed to
 */
float speed = 0;

/************************
 * Forward Declerations *
 ************************/
void velocityCallback(const std_msgs::Float64& msg);
float fscale( float originalMin, float originalMax, float newBegin, 
              float newEnd, float inputValue, float curve);

/**
 * @brief ROS node handle
 */
ros::NodeHandle nodeHandle;

/**
 * @brief ROS Subscriber for the Velocity Message
 */
ros::Subscriber<std_msgs::Float64> velocitySub(VELOCITY_TOPIC, &velocityCallback);

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
    speed = fscale(0, MAX_RAD_SEC, 0, 255, abs(msg.data), 0);
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
float fscale( float originalMin, float originalMax, float newBegin, 
              float newEnd, float inputValue, float curve)
{

    float OriginalRange = 0;
    float NewRange = 0;
    float zeroRefCurVal = 0;
    float normalizedCurVal = 0;
    float rangedValue = 0;
    boolean invFlag = 0;


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

    curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
    curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

    // Check for out of range inputValues
    if (inputValue < originalMin)
    {
        inputValue = originalMin;
    }
    if (inputValue > originalMax) 
    {
        inputValue = originalMax;
    }

    // Zero Refference the values
    OriginalRange = originalMax - originalMin;

    if (newEnd > newBegin)
    {
        NewRange = newEnd - newBegin;
    }
    else
    {
        NewRange = newBegin - newEnd;
        invFlag = 1;
    }

    zeroRefCurVal = inputValue - originalMin;
    normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

    // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
    if (originalMin > originalMax ) 
    {
        return 0;
    }

    if (invFlag == 0)
    {
        rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
    }
    else     // invert the ranges
    {  
      rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
    }

    return rangedValue;
}
