/**
 *  @file navigation_node.cpp
 *  @author shae, CS5201 Section A
 *  @date Apr 19, 2016
 *  @brief Description:
 *  @details Details:
 */



#include <ros/ros.h>
#include "navigation.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "navigation_node");
  Navigation nav_node;
  ros::Rate loop_rate(1);
  std::cout << "asdfasd" << std::endl;
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    nav_node.update();
  }
}

