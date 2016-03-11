/**
 * @file vision.cpp
 * @class Vision
 * @brief The implementation file for the Vision class
 */

#include "vision.h"

Vision::Vision() : it(nh) 
{
    visionSub = it.subscribe("/usb_cam/image_raw", 1, &Vision::imageCallback, this);
    visionPub = it.advertise("processed_image", 1);
}

void Vision::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//Get the image from the camera, and store it in frame
	try
	{
		//Switch back to MONO8
		frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	}
	//Throw an error message if the camera data isn't correct
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    return;
}

void Vision::update()
{
    visionPub.publish(frame->toImageMsg());

    return;   
}
