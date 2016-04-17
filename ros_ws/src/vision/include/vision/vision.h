/**
 * @file vision.h
 * @class Vision
 * @brief The class which converts raw camera data to usable data
 */

#ifndef VISION_H_
#define VISION_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <cv.h>
#include <vector>

class Vision
{
    private:
        /**
         * @brief The ROS node handle
         */
        ros::NodeHandle nh;

        /**
         * @brief The camera image subscriber
         */
        image_transport::Subscriber vision_sub;

        /**
         * @brief The camera image publisher
         */
        image_transport::Publisher vision_pub;

        /**
         * @brief The image tansport handle
         */
        image_transport::ImageTransport it;

        /**
         * @brief Pointer to the incoming image
         */
        cv_bridge::CvImagePtr frame;

        /**
         * @brief The callback function for the camera image
         *
         * This function accepts camera messages and throws an error if nothing is recieved
         *
         * @param msg The message which is received from the camera
         */
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    public:
        /**
         * @brief Default constructor for the Vision Class
         */
        Vision();
        
        /**
         * @brief Default Destructor for the Vision Class
         */
        ~Vision();
    
        /**
         * @brief Function used to Find the edges in the image
         */
        void edgeDetection();

        /**
         * @brief Function used to update the Camera
         */
        void update();

};

#endif
