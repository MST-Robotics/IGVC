//============================================================================
// Name        : PathFinderTutorials.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "updatepriorityqueue.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "typewkey.h"
#include <math.h>
#include "euclidiandistancehueristic.h"
#include "astar.h"

#define CONST 10
#define DONST CONST
using namespace std;

struct intWkey
{
  public:
    int m_data;
    intWkey(int data = 0)
    {
      m_data = data;
    }

    intWkey(intWkey& intwkey)
    {
      *this = intwkey;
    }

    intWkey(const intWkey& intwkey)
    {
      *this = intwkey;
    }

    const intWkey& operator =(int data)
    {
      m_data = data;
      return *this;
    }

    const intWkey& operator =(intWkey& intwkey)
    {
      m_data = intwkey.m_data;
      return *this;
    }
    const intWkey& operator =(const intWkey& intwkey)
    {
      m_data = intwkey.m_data;
      return *this;
    }

    int getKey() const
    {
      return m_data;
    }
};

int main()
{
  srand(time(NULL));
  default_compare<int> comp(false);
  /*
   MinHeap<int, default_compare<int> > heap(comp);
   for (int i = 0; i < CONST; i++)
   {
   //heap.insert((rand()%200) - 100);
   heap.insert(i - (DONST / 2));
   }
   cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
   cout << heap << endl;

   for (int i = 0; i < CONST; i++)
   {
   cout << heap.pop() << endl;
   cout << heap << endl;
   }

   UpdatePriorityQueue<intWkey, int> pq;
   intWkey keyint;
   for (size_t i = 0; i < CONST; i++)
   {
   //pq.insert((rand()%200) - 100);
   keyint = (i); //- (DONST / 2));
   pq.insert(keyint, i, i);
   cout << pq << endl;
   }

   size_t temps;
   keyint = 0;
   temps = 20; //rand() % 10;
   cout << "temps" << temps << endl;
   pq.updateCost(keyint, temps);
   cout << pq << endl;
   cout << "GOODG" << endl;
   for (size_t i = 0; i < 10; i++)
   {
   keyint = (i);
   temps = rand() % 10;
   cout << "temps" << temps << endl;
   pq.updateCost(keyint, temps);
   cout << pq << endl;
   }
   for (size_t i = 0; i < CONST; i++)
   {
   //pq.insert((rand()%200) - 100);
   //keyint = (i - (DONST / 2));
   cout << pq.pop().getKey() << endl;
   cout << pq << endl;
   }

   cout << "hello" << endl;
   //unordered_map<intWkey, h_cost> map;
   unordered_map<int, h_cost> map;
   for (size_t i = 0; i < 10; i++)
   {
   keyint = i;
   map.insert(make_pair(keyint.getKey(), i));
   cout << (map[i]) << endl;
   }
   */
  /*
   UpdatePriorityQueue<intWkey, int> rand_q;
   intWkey intwkey;
   for (size_t i = 0; i < 1000; i++)
   {
   intwkey = i;
   rand_q.insert(intwkey, i, 0);
   }
   if (!rand_q.valid_heap())
   {
   cout << "WHAT" << endl;
   }
   for (size_t i = 0; i < 1000; i++)
   {
   //std::cout << "editing " << i << std::endl;
   intwkey = i;
   int r = rand() % 1000;
   int re = rand() % 1000;
   //std::cout << "using new h val" << r << std::endl;
   rand_q.update(intwkey, re, r);
   if (!rand_q.valid_heap())
   {
   cout << "WHAT" << endl;
   return 0;
   }
   }
   if (!rand_q.valid_heap())
   {
   cout << "WHAT" << endl;
   }
   */
  EuclidianDistanceHueristic eh;
  TypeWkey<size_t> start;
  TypeWkey<size_t> goal;
  //std::vector<std::pair<size_t, size_t> > move_list = Astar(occupancyGrid[],
//      start, goal, eh);

  return 0;
}
