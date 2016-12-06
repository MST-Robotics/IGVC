/**
 *  @file updatepriorityqueue.h
 *  @author shae, CS5201 Section A
 *  @date Feb 18, 2016
 *  @brief Description:
 *  @details Details:
 */

#ifndef UPDATEPRIORITYQUEUE_H_
#define UPDATEPRIORITYQUEUE_H_

#include <unordered_map>
#include "minheap.h"

typedef double h_cost;
typedef double g_cost;
typedef double f_cost;

template<typename T, typename K>
class UpdatePriorityQueue;

template<typename T, typename K>
void swap(UpdatePriorityQueue<T, K>& first, UpdatePriorityQueue<T, K>& second);

template<typename T, typename K>
ostream& operator <<(ostream& os, const UpdatePriorityQueue<T, K>& rhs);


//T = type, K = type of key of T
template<typename T, typename K>
class UpdatePriorityQueue
{
  public:

    class compare;

    UpdatePriorityQueue();
    UpdatePriorityQueue(const UpdatePriorityQueue<T, K>& priority_queue);
    UpdatePriorityQueue(UpdatePriorityQueue<T, K> && priority_queue);
    const UpdatePriorityQueue<T, K>& operator =(
        const UpdatePriorityQueue<T, K> priority_queue);
    //~UpdatePriorityQueue();

    void insert(const T& item, const g_cost& cost, const h_cost& hueristic);
    bool updateHueristic(const T& item, const h_cost& hueristic);
    bool updateCost(const T& item, const g_cost& cost);
    const h_cost getHueristicVal(const T& item) const;
    const g_cost getCostVal(const T& item) const;
    const f_cost getTotalVal(const T& item) const;
    bool update(const T& item, const g_cost& cost, const h_cost& hueristic);
    bool remove(const T& item);
    bool find(const T& item) const;
    T pop();
    T top() const;
    size_t size();
    bool valid_heap();

    friend ostream& operator <<<T>(ostream& os,
        const UpdatePriorityQueue<T, K>& rhs);
    friend void swap<T>(UpdatePriorityQueue<T, K>& first,
        UpdatePriorityQueue<T, K>& second);

  private:
    unordered_map<K, g_cost> m_g;
    unordered_map<K, h_cost> m_h;
    unordered_map<K, size_t> m_id;
    compare m_compare;
    MinHeap<K, UpdatePriorityQueue<T, K>::compare> m_min_heap;

};

template<typename T, typename K>
class UpdatePriorityQueue<T, K>::compare
{
    /**
     * bool to reverse the comparison
     */
    bool reverse;
    /**
     * hueristic to be used in comparison
     */
  public:
    /**
     * paramaterized constructor
     * @param goal = m_goal,
     * @param revparam = reverse
     */

    unordered_map<K, g_cost>* m_g;
    unordered_map<K, h_cost>* m_h;
    compare()
    {
      reverse = false;
      m_g = nullptr;
      m_h = nullptr;
    }

    compare(UpdatePriorityQueue<T, K>::compare& comp)
    {
      *this = comp;
    }

    compare(UpdatePriorityQueue<T, K>::compare&& comp)
    {
      *this = comp;
    }
    compare(unordered_map<K, g_cost>& g, unordered_map<K, h_cost>& h,
        const bool revparam = false)
    {
      reverse = revparam;
      m_g = &g;
      m_h = &h;
    }
    const UpdatePriorityQueue<T, K>::compare& operator =(
        UpdatePriorityQueue<T, K>::compare& comp)
    {
      reverse = comp.reverse;
      m_g = comp.m_g;
      m_h = comp.m_h;
      return *this;
    }
    const UpdatePriorityQueue<T, K>::compare& operator =(
        UpdatePriorityQueue<T, K>::compare&& comp)
    {
      reverse = comp.reverse;
      m_g = comp.m_g;
      m_h = comp.m_h;
      comp.m_g = nullptr;
      comp.m_h = nullptr;
      return *this;
    }

    ~compare()
    {
      //delete m_g;
      //delete m_h;
      m_g = nullptr;
      m_h = nullptr;
    }
    /**
     * () operator to return the comparison of the hueristic of lhs and rhs
     * @param lhs left hand side ColorGraphState
     * @param rhs right hand side ColorGraphState
     * @return returns true if the hueristic of lhs is > hueristic of rhs if not reversed
     */
    bool operator()(T& lhs, T& rhs) const
    {

      size_t lhs_key = lhs.getKey();
      size_t rhs_key = rhs.getKey();
      size_t lhs_cost = (*m_g).at(lhs_key) + (*m_h).at(lhs_key);
      size_t rhs_cost = (*m_g).at(rhs_key) + (*m_h).at(rhs_key);
      if (reverse)
        return (lhs_cost > rhs_cost);
      else
        return (lhs_cost < rhs_cost);
    }
    bool operator()(K& lhs_key, K& rhs_key) const
    {
      size_t lhs_cost = (*m_g).at(lhs_key) + (*m_h).at(lhs_key);
      size_t rhs_cost = (*m_g).at(rhs_key) + (*m_h).at(rhs_key);
      //std::cout << "lhs_cost of key " << lhs_key << " is " << lhs_cost;
      //std::cout << "\n rhs_cost of key " << rhs_key << " is " << rhs_cost << std::endl;
      if (reverse)
        return (lhs_cost > rhs_cost);
      else
        return (lhs_cost < rhs_cost);
    }
};

#include "updatepriorityqueue.hpp"

#endif /* UPDATEPRIORITYQUEUE_H_ */
