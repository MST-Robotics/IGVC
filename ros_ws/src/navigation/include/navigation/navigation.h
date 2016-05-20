/**
 *  @file navigation.h
 *  @author shae, CS5201 Section A
 *  @date Apr 19, 2016
 *  @brief Description:
 *  @details Details:
 */

#ifndef NAVIGATION_INCLUDE_NAVIGATION_NAVIGATION_H_
#define NAVIGATION_INCLUDE_NAVIGATION_NAVIGATION_H_
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "astar.h"
#include "euclidiandistancehueristic.h"

class Navigation
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber omap_sub;
    nav_msgs::OccupancyGrid m_occupancy_grid;

    ros::Publisher nav_twist_pub;
    tf::TransformListener listener;
    geometry_msgs::Twist nav_twist;

    void oMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid);

  public:
    Navigation();
    void moveToLocation(std::pair<size_t, size_t> goal);
    void update();
};


#endif /* NAVIGATION_INCLUDE_NAVIGATION_NAVIGATION_H_ */
