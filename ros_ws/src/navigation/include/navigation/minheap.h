/**
 *  @file minheap.h
 *  @author shae, CS5201 Section A
 *  @date Feb 17, 2016
 *  @brief Description:
 *  @details Details:
 */

#ifndef MINHEAP_H_
#define MINHEAP_H_

#include <utility>
#include <iostream>
#include <unordered_map>
using namespace std;

template<typename T, typename comp_T>
class MinHeap;

template<typename T, typename comp_T>
ostream& operator <<(ostream& os, const MinHeap<T, comp_T>& rhs);

template<typename T, typename comp_T>
void swap(MinHeap<T, comp_T>& first, MinHeap<T, comp_T>& second);

template<typename T, typename comp_T>
class MinHeap
{
	public:
		MinHeap();
		MinHeap(comp_T& compare);
		MinHeap(comp_T&& compare);
		MinHeap(MinHeap<T, comp_T>& heap);
		MinHeap(MinHeap<T, comp_T> && heap);
		const MinHeap<T, comp_T>& operator =(MinHeap<T, comp_T> heap);

		friend void swap<T>(MinHeap<T, comp_T>& first, MinHeap<T, comp_T>& second);

		friend ostream& operator <<<T>(ostream& os, const MinHeap<T, comp_T>& rhs);

		~MinHeap();


		size_t heapifyUp(size_t i);
		size_t heapifyDown(size_t i);
		size_t insert(T item);
		void remove(size_t i);
		T pop();

        size_t heapifyUp(size_t i, unordered_map<T, size_t>& m_id);
        size_t heapifyDown(size_t i, unordered_map<T, size_t>& m_id);
        size_t insert(T item, unordered_map<T, size_t>& m_id);
        void remove(size_t i, unordered_map<T, size_t>& m_id);
		T pop(unordered_map<T, size_t>& m_id);

		T top() const;
		size_t size()const;
		comp_T getCompareObject();
		bool useCompareObject(const size_t lhs, const size_t rhs);
		bool valid_heap()const;
		const T& operator [](const size_t index)const;

	private:
		T * m_heap;
		size_t m_size;
		size_t m_end;
		size_t m_length;
		comp_T m_compare;

		void resizeLength(size_t length);

};

size_t parent(size_t i);
size_t leftChild(size_t i);
size_t rightChild(size_t i);

template<typename T>
class default_compare
{
		/**
		 * bool to reverse the comparison
		 */
		bool reverse;

	public:
		/**
		 * paramaterized constructor
		 * @param goal = m_goal,
		 * @param revparam = reverse
		 */
		default_compare(const bool revparam = false)
		{
			reverse = revparam;
		}
		/**
		 * () operator to return the comparison of the hueristic of lhs and rhs
		 * @param lhs left hand side ColorGraphState
		 * @param rhs right hand side ColorGraphState
		 * @return returns true if the hueristic of lhs is > hueristic of rhs if not reversed
		 */
		bool operator()(const T& lhs, const T& rhs) const
		{
			if (reverse)
				return (lhs > rhs);
			else
				return (lhs < rhs);
		}
};

#include "minheap.hpp"
/* namespace std */

#endif /* MINHEAP_H_ */
