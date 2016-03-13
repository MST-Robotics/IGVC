/**
 * @file vision.cpp
 * @class Vision
 * @brief The implementation file for the Vision class
 */

#include "vision.h"

//Default Constructor
Vision::Vision() : it(nh) 
{
    //Initialize Publishers and Subscribers
    vision_sub = it.subscribe("/usb_cam/image_raw", 1, &Vision::imageCallback, this);
    vision_pub = it.advertise("processed_image", 1);
}

//ROS Callback Function to get camera data
void Vision::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//Get the image from the camera, and store it in frame
	try
	{
		frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	}
	//Throw an error message if the camera data isn't correct
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    edgeDetection();
    
    return;
}

//Function used to process the image
void Vision::edgeDetection()
{

    return;
}

//Functions used to update ROS with the processed image
void Vision::update()
{
    //There is a delay in the beginning where this pointer is set to NULL, and will cause a spooky exception
    //   This check should stop this from happening
    if(frame != NULL)
    {
        vision_pub.publish(frame->toImageMsg());
    }

    return;   
}
