/**
 *  @file euclidiandistancehueristic.h
 *  @author shae, CS5201 Section A
 *  @date Apr 19, 2016
 *  @brief Description:
 *  @details Details:
 */

#ifndef EUCLIDIANDISTANCEHUERISTIC_H_
#define EUCLIDIANDISTANCEHUERISTIC_H_

#include "updatepriorityqueue.h"
#include <math.h>

class EuclidianDistanceHueristic
{
  private:
    size_t m_width;
    size_t m_height;
    size_t m_scale;
  public:
    EuclidianDistanceHueristic(const size_t width, const size_t height,
        const double scale) :
        m_width(width), m_height(height), m_scale(scale)
    {
    }
    h_cost operator ()(const TypeWkey<size_t>& start,
        const TypeWkey<size_t>& goal) const;
    virtual ~EuclidianDistanceHueristic();
};

#endif /* EUCLIDIANDISTANCEHUERISTIC_H_ */
