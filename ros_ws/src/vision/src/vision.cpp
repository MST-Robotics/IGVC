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

/*
  int xval = 0;
  int yval = 0;
  int xoffset = 600;
  int yoffset = 600;

  std::vector<cv::Point2f> points;
  points.push_back(cv::Point2f(682,573));
  points.push_back(cv::Point2f(1115,591));
  points.push_back(cv::Point2f(1140,884));
  points.push_back(cv::Point2f(555,863));
  
  
  
  
  std::vector<cv::Point2f> results;
  results.push_back(cv::Point2f(xval,yval));
  results.push_back(cv::Point2f(xval+xoffset,yval));
  results.push_back(cv::Point2f(xval+xoffset,yval+yoffset));
  results.push_back(cv::Point2f(xval,yval+yoffset));

  cv::Mat source = cv::imread("/home/robotics/IGVC/ros_ws/src/vision/src/testImage.jpg");
  
  cv::Mat h = cv::findHomography(points, results);
  
  cv::Mat out;
  
  cv::warpPerspective(source, out, h, source.size());
  
  cv::namedWindow("result",  cv::WINDOW_AUTOSIZE);
  cv::imshow("result", out);
  
  cv::waitKey(0);
*/
  
  


    cv::GaussianBlur(frame->image, frame->image, cv::Size(9, 9), 0, 0);
    cv::threshold(frame->image, frame->image, 200, 255, cv::THRESH_BINARY);

    cv::Canny(frame->image, frame->image, 50, 50);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(frame->image, lines, 1, CV_PI / 180, 50, 5, 50);
    for (size_t i = 0; i < lines.size(); i++) {
        cv::line(frame->image, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]),
                 cv::Scalar(255, 255, 255), 10, 8);
    }

    cv::circle(frame->image, cv::Point((frame->image.cols / 2), frame->image.rows),
               (frame->image.cols / 2), CV_RGB(255, 255, 255), 10);

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



//this function finds the perspective transform matrix given the angle between the two frames
cv::Mat Vision::find_perspective(float theta_x, float theta_y, float theta_z, float center_x, float center_y)
{
    //these are the matricies for the transform they could be combined but
    //I don't have my calculator or matlab and don't feel like doing it by hand

    //these are the transposes
    cv::Mat A = (cv::Mat_<float>(3, 3) <<
            1, 0, 0,
            0, cos(theta_x), sin(theta_x),
            0, -sin(theta_x),  cos(theta_x));

    cv::Mat B = (cv::Mat_<float>(3, 3) <<
            cos(theta_y), 0, -sin(theta_y),
            0, 1, 0,
            sin(theta_y), 0, cos(theta_y));

    cv::Mat C = (cv::Mat_<float>(3, 3) <<
            cos(theta_z), sin(theta_z), 0,
            -sin(theta_z),  cos(theta_z), 0,
            0, 0, 1);

    //shift to center
    cv::Mat D = (cv::Mat_<float>(3, 3) <<
            1, 0, -center_x,
            0, 1, -center_y,
            0, 0, 1);

    cv::Mat T = A*B*C*D;

    //shift back by sclaed ammounts
    cv::Mat E = (cv::Mat_<float>(3, 3) << (T.at<float>(2,2) / T.at<float>(0,0) ), 0, center_x ,
            0, (T.at<float>(2,2) / T.at<float>(1,1) ), center_y  , 0, 0, 1);

    T = E*T;

    return T;
}
