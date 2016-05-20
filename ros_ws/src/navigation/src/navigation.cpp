/**
 *  @file navigation.cpp
 *  @author shae, CS5201 Section A
 *  @date Apr 19, 2016
 *  @brief Description:
 *  @details Details:
 */

#include "navigation.h"

void Navigation::oMapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid)
{
  //size_t width = (*occupancy_grid).info.width;
  //size_t height = (*occupancy_grid).info.height;
  m_occupancy_grid = *occupancy_grid;
  //std::cout << "Width " << width << " Height " << height << std::endl;
}

Navigation::Navigation()
{
  omap_sub = nh.subscribe<nav_msgs::OccupancyGrid> ("map", 1, &Navigation::oMapCallback, this);
  nav_twist_pub = nh.advertise<geometry_msgs::Twist>("auto_twist", 1);
}

void Navigation::moveToLocation(std::pair<size_t, size_t> goal)
{
  // Get robots current location
  geometry_msgs::PointStamped goal_robot;
  tf::StampedTransform transform;
  goal_robot.header.frame_id = "map";
  goal_robot.point.x = goal.first;
  goal_robot.point.y = goal.second;

  try
  {
    listener.transformPoint("/base_link", goal_robot, goal_robot);
    double angle = atan2(goal_robot.point.y, goal_robot.point.x);
    double dist = sqrt(pow(goal_robot.point.y, 2) + pow(goal_robot.point.x, 2));
    cout << angle << endl;

    if(dist > .5)
    {
      if (abs(angle) > .1)
      {
        nav_twist.linear.x = 0;
        if (angle < 0)
          nav_twist.angular.z = -.25;
        else if (angle > 0)
          nav_twist.angular.z = .25;
      }
      else
      {
        nav_twist.angular.z = 0;
        nav_twist.linear.x = .5;
      }
    }
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void Navigation::update()
{
  size_t width = m_occupancy_grid.info.width;
  size_t height = m_occupancy_grid.info.height;
  if (width && height)
  {
    size_t x = 3;
    size_t y = 4;
    size_t xg = 100;
    size_t yg = 70;
    std::cout << "Width " << width << " Height " << height << std::endl;
    TypeWkey<size_t> start = (x % width) + (y * width);
    TypeWkey<size_t> goal = (xg % width) + (yg * width);
    EuclidianDistanceHueristic eh(width, height, 1.0);

    moveToLocation(Astar<size_t, EuclidianDistanceHueristic>(m_occupancy_grid, start, goal, eh)[0]);
  }

  nav_twist_pub.publish(nav_twist);
}
