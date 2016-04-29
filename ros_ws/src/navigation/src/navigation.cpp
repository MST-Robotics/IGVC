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
    Astar<size_t, EuclidianDistanceHueristic>(m_occupancy_grid, start, goal,
        eh);
  }
}
