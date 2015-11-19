/**
 * @file vision.cpp
 * @class Vision
 * @brief The implementation file of the Vision class
 */

#include "vision.h"

Vision::Vision() : it(nh)
{
    im_sub = it.subscribe(VISION_INCOMING_TOPIC, 1, &Vision::imageCallback, this);
    im_pub = it.advertise(VISION_OUTGOING_TOPIC, 1);
}

void Vision::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Ensure the image is of the expected encoding and throw an error if not
//    try
//    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

    // Do processing of image here

    //Publish processed image
    im_pub.publish(cv_ptr->toImageMsg());
}
