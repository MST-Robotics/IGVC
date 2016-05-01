/**
 *  @file astar.hpp
 *  @author shae, CS5201 Section A
 *  @date Apr 19, 2016
 *  @brief Description:
 *  @details Details:
 */


template<typename typekey, typename Heuristic>
std::vector<std::pair<size_t, size_t>> Astar(nav_msgs::OccupancyGrid o_grid,
    TypeWkey<typekey> start, TypeWkey<typekey> goal, const Heuristic& heuristic)
{
  std::cout << "Starting Astar" << std::endl;
  //set of moves to be returned
  std::vector<std::pair<size_t, size_t>> moves;
  //occupancy grid width
  size_t width = o_grid.info.width;
  //occupancy grid height
  size_t height = o_grid.info.height;
  //scale cost of a single cardinal move
  const double scale = 1;
  //path cost for start
  g_cost start_g = 0;
  //heuristic cost for start
  h_cost start_h = heuristic(start, goal);
  //open priority queue, with removal and updating
  UpdatePriorityQueue<TypeWkey<typekey>, size_t> openPQ;
  //closed set map, maps the key for the type and the f cost for that type
  unordered_map<typekey, f_cost> closedSet;
  //map that maps parent with given key index
  unordered_map<typekey, typekey > parents;
  // parents.insert(std::make_pair(start, start)); //parents[start] = undefined, we set the parent as itself
  //inserts the first item
  openPQ.insert(start, start_g, start_h); //openQ.add(start, f_score[start])

  //u = current node
  TypeWkey<typekey> u;
  //v = next node
  TypeWkey<typekey> v;
  //set of adjacent moves to u
  std::vector<std::pair<TypeWkey<typekey>, g_cost> > adj_moves;
  //std::cout << "HOLA" <<std::endl;
  //while the priority queue is not empty
  while (openPQ.size())
  {
    //std::cout << "While Priority Queue is Not Empty, PriorityQueue size " << openPQ.size() << std::endl;
    //total score, g_score + h_score for u
    f_cost top_score = openPQ.getTotalVal(openPQ.top());
   // std::cout << "we got the top score " << top_score << endl;
    //poping off u from the priority queue
    u = openPQ.pop();  // Source node in first case
   // std::cout << "popped " << top_score << endl;
    if (u == goal)
    {
      //break and return current move list;
      return reconstruct_path<typekey>(moves, parents, u, width, height);
    }
    //std::cout << "not goal yet " << std::endl;
    //if it isn't the goal we move the position to the closed set
    closedSet.insert(std::make_pair(u.getKey(), top_score));

   // std::cout << "Inserting done " << std::endl;
    //clear previous adjacent moves
    adj_moves.clear();
   // std::cout << "clearing moves " << std::endl;
    //NORTH
    typekey temp_key = u.getKey();
    //Cardinal direction keys
    typekey north_key = temp_key - width;
    typekey south_key = temp_key + width;
    typekey east_key = temp_key + 1;
    typekey west_key = temp_key - 1;
    //Diagonal direction keys
    typekey north_w_key = north_key - 1;
    typekey north_e_key = north_key + 1;
    typekey south_w_key = south_key - 1;
    typekey south_e_key = south_key + 1;
    //diagonal distance
    g_cost ddist = openPQ.getCostVal(u) + (scale * sqrt(2));
    //straight distance
    g_cost sdist = openPQ.getCostVal(u) + scale;
    //north lookup
    if (temp_key >= width)
    {
      adj_moves.push_back(std::make_pair(TypeWkey<typekey>(north_key), sdist));
      //north west lookup
      if (temp_key % width > 0)
      {
        adj_moves.push_back(std::make_pair(TypeWkey<typekey>(north_w_key), ddist));
      }
      //north east lookup
      if (temp_key % width < (width - 1))
      {
        adj_moves.push_back(std::make_pair(TypeWkey<typekey>(north_e_key), ddist));
      }
    }
    //south
    if (temp_key <= (width - 1) * (height))
    {
      adj_moves.push_back(std::make_pair(TypeWkey<typekey>(south_key), sdist));
      //south west lookup
      if (temp_key % width > 0)
      {
        adj_moves.push_back(std::make_pair(TypeWkey<typekey>(south_w_key), ddist));
      }
      //south east lookup
      if (temp_key % width < (width - 1))
      {
        adj_moves.push_back(std::make_pair(TypeWkey<typekey>(south_e_key), ddist));
      }
    }

    // west lookup
    if (temp_key % width > 0)
    {
      adj_moves.push_back(std::make_pair(TypeWkey<typekey>(west_key), sdist));
    }
    // east lookup
    if (temp_key % width < (width - 1))
    {
      adj_moves.push_back(std::make_pair(TypeWkey<typekey>(east_key), sdist));
    }

    //std::cout << "Done generating Adjacency Moves" << std::endl;
    //going through all possible adjacent moves
    for (size_t i = 0; i < adj_moves.size(); i++)     // where v is still in PQ.
    {
      //adjacent move
      v = adj_moves[i].first;
      //std::cout << "Going through adjacent moves " << i << std::endl;
      //if it is inside the closed set we can ignore it. Otherwise continue
      if (closedSet.find(v.getKey()) == closedSet.end())
      {
        //if the probability of blockage in the probability grid is below 2% (1%, 0%, -1 (not explored)%)
        if (o_grid.data[v.getKey()] < 2)
        {
          //path cost of v
          g_cost path_cost = adj_moves[i].second; // determined by edge length and weight
          //heuristic value of v
          h_cost h_val = heuristic(v.getKey(), goal);

          //if it wasn't already in the openset
          if (!openPQ.find(v))
          {
            //we insert it
            parents.insert(std::make_pair(v.getKey(), u.getKey()));
            openPQ.insert(v, path_cost, h_val);
          }
          //if it was already in the openset, and we found that this path is shorter
          //than the one already stored
          else if (path_cost < openPQ.getCostVal(v)) // A shorter path to v has been found
          {
            //inserting parent (where v is the child and u is the parent)
            parents.insert(std::make_pair(v.getKey(), u.getKey()));
            //update the value
            openPQ.update(v, path_cost, h_val);
          }
        }
      }
    }
    //std::cout << "Done assessing adjacent " << std::endl;
  }
  return moves;
}

template<typename typekey>
const std::vector<std::pair<size_t, size_t>>& reconstruct_path(
    std::vector<std::pair<size_t, size_t>>& moves,
    const unordered_map<typekey ,typekey >& parents,
    const TypeWkey<typekey>& current, size_t width, size_t hieght)
{
  TypeWkey<typekey> path_current = current;
  std::cout << "finding parents" << std::endl;
  do
  {

    moves.push_back(std::make_pair(path_current.getKey() % width, (path_current.getKey() / width)));
    path_current = parents.at(path_current.getKey());
  } while (parents.find(path_current.getKey()) != parents.end());
  return moves;
}


