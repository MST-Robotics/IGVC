/**
 *  @file minheap.cpp
 *  @author shae, CS5201 Section A
 *  @date Apr 19, 2016
 *  @brief Description:
 *  @details Details:
 */

#include "minheap.h"

size_t parent(size_t i)
{

  return (i ? (i - 1) : i) / 2;
}

size_t leftChild(size_t i)
{
  return (i * 2) + 1;
}

size_t rightChild(size_t i)
{
  return (i * 2) + 2;
}


