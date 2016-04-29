/**
 *  @file euclidiandistancehueristic.cpp
 *  @author shae, CS5201 Section A
 *  @date Apr 19, 2016
 *  @brief Description:
 *  @details Details:
 */

#include "euclidiandistancehueristic.h"

h_cost EuclidianDistanceHueristic::operator ()(const TypeWkey<std::size_t>& start,
    const TypeWkey<std::size_t>& goal) const
{
  std::size_t start_x = start.getKey() % m_width;
  std::size_t start_y = start.getKey() / m_width;
  std::size_t goal_x = goal.getKey() % m_width;
  std::size_t goal_y = goal.getKey() / m_width;
  return m_scale
      * sqrt(
          ((goal_x - start_x) * (goal_x - start_x))
              + ((goal_y - start_y) * (goal_y - start_y)));
}


