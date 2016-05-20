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

void onClick(int event, int x, int y, int d, void* ptr)
{
    if(event = CV_EVENT_LBUTTONDOWN) {
        cv::Point2f pt = cv::Point2f(x, y);
        static_cast<std::vector<cv::Point2f>*>(ptr)->push_back(pt);
    }
}

//Function used to process the image
void Vision::edgeDetection()
{
    //find the perspective transform
    if(false)
    {
        cv::Mat transform_matrix_to_save;

        std::vector<cv::Point2f> src;
        cv::Point2f src2[4];
        //cv::Point2f dst[4] = {cv::Point2f(0,0),cv::Point2f(0,17),cv::Point2f(22,17),cv::Point2f(22,0)};

        cv::namedWindow("Calibrate");
        cv::imshow("Calibrate", frame->image);
        cv::setMouseCallback("Calibrate", onClick, (&src));
        cv::waitKey(0);

        cv::destroyWindow("Calibrate");
        if(src.size() >=4)
        {
            for(int i = 0; i < 4; i++)
            {
                src2[i] = src[i];
            }

            float widthA = sqrt((pow((src2[2].x - src2[3].x),2)+pow((src2[2].y - src2[3].y),2)));
            float widthB = sqrt((pow((src2[0].x - src2[1].x),2)+pow((src2[0].y - src2[1].y),2)));
            float maxWidth = (widthA > widthB ? widthA : widthB);
            float heightA = sqrt((pow((src2[0].x - src2[2].x),2)+pow((src2[0].y - src2[2].y),2)));
            float heightB = sqrt((pow((src2[1].x - src2[3].x),2)+pow((src2[1].y - src2[3].y),2)));
            float maxHeight = (heightA > heightB ? heightA : heightB);

            cv::Point2f dst[4] = {cv::Point2f(0,0),cv::Point2f(maxWidth-1,0),cv::Point2f(maxWidth-1,maxHeight-1),cv::Point2f(0,maxHeight-1)};

            transform_matrix_to_save = cv::getPerspectiveTransform(dst, src2);
            std::cout << transform_matrix_to_save << std::endl;
        }
    }
    cv::Mat M = (cv::Mat_<float>(3, 3) <<
            1950.677379756595, -440.8234218811642, 805.9999999999999,
    504.5687519288051, -115.4016385761404, 211.0000000000016,
    2.414204671037371, -0.5469273216807602, 1);
    cv::warpPerspective(frame->image,frame->image,M,frame->image.size());
/*
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
*/
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
