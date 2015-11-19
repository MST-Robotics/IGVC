/**
 * @file vision.h
 * @class Vision
 * @brief A class which handles vision processing
 *
 * This class is meant to hold the majority of the vision processing functionality on the robot and will be set up as a
 * ros node which subscribes to image messages and outputs an updated image message.
 */

#ifndef VISION_H
#define VISION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "constants.h"

class Vision {
private:
    /**
     * @brief The ros node handle for the vision class
     */
    ros::NodeHandle nh;

    /**
     * @brief The image transport handler
     */
    image_transport::ImageTransport it;

    /**
     * @brief The image subscriber for ros
     *
     * The subscriber to images from ros. It will get images from a camera feed for processing.
     */
    image_transport::Subscriber im_sub;

    /**
     * @brief The image publisher for after the image has been processed
     */
    image_transport::Publisher im_pub;

    /**
     * @brief The working copy of the opencv image
     *
     * This will be the image that is received from the camera. This contains the opencv format of the image which
     * will have to be converted back to the ros version before being published.
     */
    cv_bridge::CvImagePtr cv_ptr;

    /**
     * @brief The callback function for image messages
     *
     * This function will take image messages and convert them to the opencv equivalent.  The function will also do any
     * processing necessary to the image so that it can be published.
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public:
    /**
     * @brief The default constructor for the Vision class
     *
     * This constructor will do the initial ros setup for the class.
     */
    Vision();
};


#endif //PROJECT_VISION_H
