/**
 *  @file updatepriorityqueue.hpp
 *  @author shae, CS5201 Section A
 *  @date Feb 18, 2016
 *  @brief Description:
 *  @details Details:
 */

template<typename T, typename K>
UpdatePriorityQueue<T, K>::UpdatePriorityQueue()
{
  m_g.clear();
  m_h.clear();
  m_id.clear();
  m_compare = UpdatePriorityQueue<T, K>::compare(m_g, m_h, false);
  m_min_heap = MinHeap<K, UpdatePriorityQueue<T, K>::compare>(m_compare);
}

template<typename T, typename K>
UpdatePriorityQueue<T, K>::UpdatePriorityQueue(
    const UpdatePriorityQueue<T, K>& priority_queue)
{
  m_g = priority_queue.m_g;
  m_h = priority_queue.m_h;
  m_id = priority_queue.m_id;
  m_compare = priority_queue.m_compare;
  m_min_heap = priority_queue.m_min_heap;
}

template<typename T, typename K>
UpdatePriorityQueue<T, K>::UpdatePriorityQueue(
    UpdatePriorityQueue<T, K> && priority_queue) :
    UpdatePriorityQueue<T, K>()
{
  swap(*this, priority_queue);
}

template<typename T, typename K>
const UpdatePriorityQueue<T, K>& UpdatePriorityQueue<T, K>::operator =(
    const UpdatePriorityQueue<T, K> priority_queue)
{
  swap(*this, priority_queue);
  return *this;
}

//~UpdatePriorityQueue();

template<typename T, typename K>
void UpdatePriorityQueue<T, K>::insert(const T& item, const g_cost& cost,
    const h_cost& hueristic)
{
  K key = item.getKey();
  //unordered_map<K, g_cost> map;
  //map.insert(make_pair(key, cost));
  //cout << "map" << map[key] << endl;

  g_cost g = cost;
  h_cost h = hueristic;
  //cout << g << endl;
  //cout << h << endl;

  m_g.insert(make_pair(key, g));
  //cout << "m_g" << m_g[key] << endl;
  //(*m_g.find(key)).second = g;
  //cout << "asdf " << make_pair(key, cost).first << "asdf "
  //		<< make_pair(key, cost).second << endl;

  m_h.insert(make_pair(key, h));
  //(*m_h.find(key)).second = h;
  m_id.insert(make_pair(key, size() - 1));
  size_t heap_index = m_min_heap.insert(key, m_id);
  m_id[key] = heap_index;
  //cout << "H" << hueristic << endl;
  //cout << "m_g" << m_g[key] << endl;
  //cout << "m_g" << (*m_g.find(key)).second << endl;
  //cout << "m_h" << m_h[key] << endl;
  //cout << "m_id" << m_id[key] << endl;
}

template<typename T, typename K>
bool UpdatePriorityQueue<T, K>::updateHueristic(const T& item,
    const h_cost& hueristic)
{
  K key = item.getKey();
  try
  {
    m_h.at(key) = hueristic;
  }
  catch (const std::out_of_range& oor)
  {
    std::cerr << "Out of Range error: " << oor.what() << '\n';
    return false;
  }
  size_t& index_value = m_id.at(key);
  index_value = m_min_heap.heapifyUp(index_value, m_id);
  index_value = m_min_heap.heapifyDown(index_value, m_id);
  m_id.at(key) = index_value;
  return true;
}

template<typename T, typename K>
bool UpdatePriorityQueue<T, K>::updateCost(const T& item, const g_cost& cost)
{
  K key = item.getKey();
  try
  {
    m_g.at(key) = cost;
  }
  catch (const std::out_of_range& oor)
  {
    std::cerr << "Out of Range error: " << oor.what() << '\n';
    return false;
  }
  size_t& index_value = m_id.at(key);
  index_value = m_min_heap.heapifyUp(index_value, m_id);
  index_value = m_min_heap.heapifyDown(index_value, m_id);
  //m_id.at(key) = index_value;
  return true;
}
template<typename T, typename K>
const h_cost UpdatePriorityQueue<T, K>::getHueristicVal(const T& item) const
{
  K key = item.getKey();
  return m_h.at(key);
}
template<typename T, typename K>
const g_cost UpdatePriorityQueue<T, K>::getCostVal(const T& item) const
{
  K key = item.getKey();
  return m_g.at(key);
}

template<typename T, typename K>
const f_cost UpdatePriorityQueue<T, K>::getTotalVal(const T& item) const
{
  K key = item.getKey();
  return m_g.at(key) + m_h.at(key);
}

template<typename T, typename K>
bool UpdatePriorityQueue<T, K>::update(const T& item, const g_cost& cost,
    const h_cost& hueristic)

{
  K key = item.getKey();
  //std::cout << "KEY " << key << std::endl;
  try
  {
    m_g.at(key) = cost;
    m_h.at(key) = hueristic;
  }
  catch (const std::out_of_range& oor)
  {
    std::cerr << "Out of Range error: " << oor.what() << '\n';
    return false;
  }
  //std::cout << "mid " << std::endl;
  //std::cout << "index " <<  m_id.at(key) << std::endl;
  size_t& index_value = m_id.at(key);
  //std::cout << "fval "<<  m_g.at(key) +  m_h.at(key) << std::endl;
  //std::cout << "index val "<< index_value << std::endl;
  index_value = m_min_heap.heapifyUp(index_value, m_id);
  //std::cout << "after up position " << m_id.at(key) << std::endl;
  index_value = m_min_heap.heapifyDown(index_value, m_id);
  //std::cout << "after down position " << m_id.at(key) << std::endl;
  //std::cout << "endmid " << std::endl;
  //m_id.at(key) = index_value;
  return true;
}

template<typename T, typename K>
bool UpdatePriorityQueue<T, K>::remove(const T& item)
{
  K key = item.getKey();
  size_t index_value;
  try
  {
    index_value = m_id.at(key);
  }
  catch (const std::out_of_range& oor)
  {
    //std::cerr << "Out of Range error: " << oor.what() << '\n';
    return false;
  }
  m_min_heap.remove(index_value, m_id);
  m_h.erase(key);
  m_g.erase(key);
  m_id.erase(key);
  return true;
}

template<typename T, typename K>
bool UpdatePriorityQueue<T, K>::find(const T& item) const
{
  K key = item.getKey();
  typename std::unordered_map<K, size_t>::const_iterator got = m_id.find(key);

  return (got != m_id.end()) ? true : false;
}

template<typename T, typename K>
T UpdatePriorityQueue<T, K>::pop()
{
  return m_min_heap.pop(m_id);
}

template<typename T, typename K>
T UpdatePriorityQueue<T, K>::top() const
{
  return m_min_heap.top();
}

template<typename T, typename K>
size_t UpdatePriorityQueue<T, K>::size()
{
  return m_min_heap.size();
}
template<typename T, typename K>
bool UpdatePriorityQueue<T, K>::valid_heap()
{
  size_t m_end = size() - 1;
  size_t j;
  compare t_compare = m_min_heap.getCompareObject();
  for (size_t i = 0; i < size(); i++)
  {

    if (((2 * i) + 1) < (m_end))
    {
      size_t left = leftChild(i);
      size_t right = rightChild(i);
      j = m_min_heap.useCompareObject(left, right) ? left : right;
    }
    else
    {
      j = ((2 * i) + 1 <= m_end) ? ((2 * i) + 1) : i;
    }
    if (m_min_heap.useCompareObject(j, i))
    {
      K key_i = m_min_heap[i];
      K key_j = m_min_heap[j];

      //std::cout << m_min_heap.useCompareObject(j, i) << std::endl;
      //std::cout << "comparing i position " << i << " with j position " << j << std::endl;
      //std::cout << "comparing i key " << m_min_heap[i] << " with j key"
       //   << m_min_heap[j] << std::endl;
      //std::cout << "comparing h[i] " << m_h.at(key_i) << " with h[j] " << m_h.at(key_j)
       //   << std::endl;
      //std::cout << "comparing g[i] " << m_g.at(key_i) << " with g[j] " << m_g.at(key_j)
       //   << std::endl;
      //std::cout << "comparing f[i] " << m_g.at(key_i) + m_h.at(key_i) << " with f[j] "
        //  << m_g.at(key_j) + m_h.at(key_j) << std::endl;
      for (size_t k = 0; k < size(); k++)
      {
        K key_k = m_min_heap[k];
        //std::cout << m_min_heap[k] << " : " << m_g.at(key_k) + m_h.at(key_k) << std::endl;
      }
      return false;
    }
  }
  return true;
  return m_min_heap.valid_heap();
}

template<typename T, typename K>
ostream& operator <<(ostream& os, const UpdatePriorityQueue<T, K>& rhs)
{
  os << rhs.m_min_heap;
  return os;
}

template<typename T, typename K>
void swap(UpdatePriorityQueue<T, K>& first, UpdatePriorityQueue<T, K>& second)
{
  swap(first.m_g, second.m_g);
  swap(first.m_h, second.m_h);
  swap(first.m_id, second.m_id);
  swap(first.m_compare, second.m_compare);
  swap(first.m_min_heap, second.m_min_heap);
}
