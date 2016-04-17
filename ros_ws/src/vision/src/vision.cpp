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

//Destructor
Vision::~Vision()
{
  
}

//ROS Callback Function to get camera data
void Vision::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//Get the image from the camera, and store it in frame
	try
	{
		frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
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
    cv::threshold(frame->image,frame->image,127,255,cv::THRESH_BINARY);
    cv::GaussianBlur(frame->image, frame->image, cv::Size(9,9), 0, 0);
    cv::Canny(frame->image, frame->image, 100, (100 * 3), 3);
    
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(frame->image, lines, 1, CV_PI/180, 80, 30, 10);
    for(size_t i = 0; i < lines.size(); i++)
    {
        cv::line(frame->image, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255,255,255), 3, 8);
    }
    
    cv::circle(frame->image, cv::Point((frame->image.cols/2), frame->image.rows), 
                (frame->image.cols/2), CV_RGB(255,255,255), 10);
    
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
