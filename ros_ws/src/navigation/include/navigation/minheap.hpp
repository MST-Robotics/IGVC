/**
 *  @file minheap.hpp
 *  @author shae, CS5201 Section A
 *  @date Feb 17, 2016
 *  @brief Description:
 *  @details Details:
 */

template<typename T, typename comp_T>
MinHeap<T, comp_T>::MinHeap()
{
  m_size = 0;
  m_end = 0;
  m_length = 0;
  m_heap = nullptr;
  m_compare = comp_T();
}

template<typename T, typename comp_T>
MinHeap<T, comp_T>::MinHeap(comp_T& compare)
{
  m_size = 0;
  m_end = 0;
  m_length = 0;
  m_heap = nullptr;
  m_compare = compare;
}

template<typename T, typename comp_T>
MinHeap<T, comp_T>::MinHeap(comp_T&& compare)
{
  m_size = 0;
  m_end = 0;
  m_length = 0;
  m_heap = nullptr;
  m_compare = compare;
}

template<typename T, typename comp_T>
MinHeap<T, comp_T>::MinHeap(MinHeap<T, comp_T>& heap)
{
  m_size = heap.m_size;
  m_end = heap.m_end;
  m_length = heap.m_length;
  std::copy(m_heap, m_heap + m_size, heap.m_heap);
  m_compare = heap.m_compare;
}

template<typename T, typename comp_T>
MinHeap<T, comp_T>::MinHeap(MinHeap<T, comp_T> && heap) :
    MinHeap<T, comp_T>()
{
  swap(*this, heap);
}

template<typename T, typename comp_T>
const MinHeap<T, comp_T>& MinHeap<T, comp_T>::operator =(
    MinHeap<T, comp_T> heap)
{
  swap(*this, heap);
  return *this;
}

template<typename T, typename comp_T>
MinHeap<T, comp_T>::~MinHeap()
{
  delete[] m_heap;
  m_heap = nullptr;
}

template<typename T, typename comp_T>
size_t MinHeap<T, comp_T>::heapifyUp(size_t i)
{
  size_t j = 0;
  while (i > 0)
  {
    j = parent(i);
    if (m_compare(m_heap[i], m_heap[j]))
    {
      std::swap(m_heap[i], m_heap[j]);
      i = j;
    }
    else
    {
      return i;
    }
  }
  return i;
}

template<typename T, typename comp_T>
size_t MinHeap<T, comp_T>::heapifyUp(size_t i, unordered_map<T, size_t>& m_id)
{
  size_t j = 0;
  while (i > 0)
  {
    j = parent(i);
    if (m_compare(m_heap[i], m_heap[j]))
    {
      //std::cout << "swapping at i  = " << i << " and j = " << j << std::endl;
      //std::cout << m_heap[i] <<" , " << m_heap[j] << std::endl;
      std::swap(m_heap[i], m_heap[j]);
      //std::cout << "i " << std::endl;
      //std::cout << m_id.at(m_heap[i]) << std::endl;
      //std::cout << "j " << std::endl;
      //std::cout << m_id.at(m_heap[j]) << std::endl;
      std::swap(m_id.at(m_heap[i]), m_id.at(m_heap[j]));
      //std::cout << m_heap[i] <<" , " << m_heap[j] << std::endl;
      i = j;
    }
    else
    {
      return i;
    }
  }
  return i;
}

template<typename T, typename comp_T>
size_t MinHeap<T, comp_T>::heapifyDown(size_t i, unordered_map<T, size_t>& m_id)
{
  size_t j = 0;

  while (((2 * i) + 1) <= (m_end))
  {

    if (((2 * i) + 1) < (m_end))
    {
      size_t left = leftChild(i);
      size_t right = rightChild(i);
      j = m_compare(m_heap[left], m_heap[right]) ? left : right;
    }
    else
    {
      j = (2 * i) + 1;
    }
    if (m_compare(m_heap[j], m_heap[i]))
    {
      //std::cout << "swapping at i  = " << i << " and j = " << j << std::endl;
      //std::cout << m_heap[i] <<" , " << m_heap[j] << std::endl;
      //cout << "hello " << ((2 * i) + 1) << endl;
      std::swap(m_heap[i], m_heap[j]);
      std::swap(m_id.at(m_heap[i]), m_id.at(m_heap[j]));
      //std::cout << m_heap[i] <<" , " << m_heap[j] << std::endl;
      //std::cout << std::endl;
      i = j;
    }
    else
    {
      return i;
    }
  }
  return i;
}

template<typename T, typename comp_T>
size_t MinHeap<T, comp_T>::heapifyDown(size_t i)
{
  size_t j = 0;

  while (((2 * i) + 1) <= (m_end))
  {

    if (((2 * i) + 1) < (m_end))
    {
      size_t left = leftChild(i);
      size_t right = rightChild(i);
      j = m_compare(m_heap[left], m_heap[right]) ? left : right;
    }
    else
    {
      j = (2 * i) + 1;
    }
    if (m_compare(m_heap[j], m_heap[i]))
    {
      std::swap(m_heap[i], m_heap[j]);
      i = j;
    }
    else
    {
      return i;
    }
  }
  return i;
}

template<typename T, typename comp_T>
size_t MinHeap<T, comp_T>::insert(T item)
{
  if ((m_size + 1) >= m_length)
  {
    resizeLength((m_size + 1) * 2);
  }
  m_end = m_size;
  m_size += 1;
  m_heap[m_end] = item;

  return heapifyUp(m_end);
}

template<typename T, typename comp_T>
size_t MinHeap<T, comp_T>::insert(T item, unordered_map<T, size_t>& m_id)
{
  if ((m_size + 1) >= m_length)
  {
    resizeLength((m_size + 1) * 2);
  }
  m_end = m_size;
  m_size += 1;
  m_heap[m_end] = item;

  return heapifyUp(m_end, m_id);
}

template<typename T, typename comp_T>
void MinHeap<T, comp_T>::remove(size_t i, unordered_map<T, size_t>& m_id)
{
  try
  {
    if (!(i < m_size))
    {
      throw i;
    }
    std::swap(m_heap[i], m_heap[m_end]);
    std::swap(m_id.at(m_heap[i]), m_id.at(m_heap[m_end]));
    if (m_size > 0)
    {
      m_size -= 1;
      m_end -= m_end ? 1 : 0;
    }
    i = heapifyUp(i,  m_id);
    heapifyDown(i, m_id);

    if (((m_size) <= m_length / 2))
    {
      resizeLength(m_length / 2);
    }
  }
  catch (size_t i)
  {
    cout << "error, i out of bounds, i = " << i << endl;
  }
}



template<typename T, typename comp_T>
void MinHeap<T, comp_T>::remove(size_t i)
{
  try
  {
    if (!(i < m_size))
    {
      throw i;
    }
    std::swap(m_heap[i], m_heap[m_end]);

    if (m_size > 0)
    {
      m_size -= 1;
      m_end -= m_end ? 1 : 0;
    }
    i = heapifyUp(i);
    heapifyDown(i);

    if (((m_size) <= m_length / 2))
    {
      resizeLength(m_length / 2);
    }
  }
  catch (size_t i)
  {
    cout << "error, i out of bounds, i = " << i << endl;
  }
}

template<typename T, typename comp_T>
T MinHeap<T, comp_T>::pop()
{
  T val;
  if (m_size > 0)
  {
    val = m_heap[0];
    remove(0);
  }
  else
  {
    cout << "NO TOP " << m_size << endl;
  }
  return val;
}

template<typename T, typename comp_T>
T MinHeap<T, comp_T>::pop(unordered_map<T, size_t>& m_id)
{
  T val;
  if (m_size > 0)
  {
    val = m_heap[0];
    remove(0, m_id);
  }
  else
  {
    cout << "NO TOP " << m_size << endl;
  }
  return val;
}

template<typename T, typename comp_T>
T MinHeap<T, comp_T>::top() const
{
  T val;
  if (m_size > 0)
  {
    val = m_heap[0];
  }
  else
  {
    cout << "NO TOP " << m_size << endl;
  }
  return val;
}

template<typename T, typename comp_T>
size_t MinHeap<T, comp_T>::size() const
{
  return m_size;
}

template<typename T, typename comp_T>
comp_T MinHeap<T, comp_T>::getCompareObject()
{
  return m_compare;
}

template<typename T, typename comp_T>
void MinHeap<T, comp_T>::resizeLength(size_t length)
{
  T *temp = new T[length];
  for (size_t i = 0; i < m_size; i++)
  {
    temp[i] = m_heap[i];
  }
  delete[] m_heap;
  m_heap = temp;
  m_length = length;
}

template<typename T, typename comp_T>
bool MinHeap<T, comp_T>::valid_heap() const
{
  size_t j;
  for (size_t i = 0; i < m_size - 1; i++)
  {

    if (((2 * i) + 1) < (m_end))
    {
      size_t left = leftChild(i);
      size_t right = rightChild(i);
      j = m_compare(m_heap[left], m_heap[right]) ? left : right;
    }
    else
    {
      j = ((2 * i) + 1 <= m_end) ? ((2 * i) + 1) : i;
    }
    if (m_compare(m_heap[j], m_heap[i]))
    {
      std::cout << m_compare(m_heap[j], m_heap[i]) << std::endl;
      std::cout << "comparing i " << i << " with j " << j << std::endl;
      std::cout << "comparing i " << m_heap[i] << " with j " << m_heap[j]
          << std::endl;
      for (size_t k = 0; k < m_size; k++)
      {
        std::cout << m_heap[k] << std::endl;
      }
      return false;
    }
  }
  return true;
}

template<typename T, typename comp_T>
bool MinHeap<T, comp_T>::useCompareObject(const size_t lhs, const size_t rhs)
{
  return m_compare(m_heap[lhs], m_heap[rhs]);
}

template<typename T, typename comp_T>
const T& MinHeap<T, comp_T>::operator [](const size_t index) const
{
  try
  {
    if (!(index < size()))
    {
      throw index;
    }
  }
  catch (size_t i)
  {
    cout << "error, i out of bounds, i = " << i << endl;
  }
  return m_heap[index];
}

template<typename T, typename comp_T>
void swap(MinHeap<T, comp_T>& first, MinHeap<T, comp_T>& second)
{
  std::swap(first.m_size, second.m_size);
  std::swap(first.m_length, second.m_length);
  std::swap(first.m_heap, second.m_heap);
  std::swap(first.m_compare, second.m_compare);
}

template<typename T, typename comp_T>
ostream& operator <<(ostream& os, const MinHeap<T, comp_T>& rhs)
{
  for (size_t i = 0; i < rhs.m_size; i++)
  {
    os << rhs.m_heap[i] << ", ";
  }
  return os;
}

