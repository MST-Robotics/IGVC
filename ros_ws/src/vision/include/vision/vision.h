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

class Vision
{
    private:
        /**
         * @brief Initialize the Ros Node Handle
         */
        ros::NodeHandle nh;

        image_transport::Subscriber visionSub;
        image_transport::Publisher visionPub;

        image_transport::ImageTransport it;

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        cv_bridge::CvImagePtr frame;

    public:
        /**
         * @brief Default constructor for the Vision Class
         */
        Vision();

        /**
         * @brief Function used to update the Camera
         */
        void update();

};

#endif
