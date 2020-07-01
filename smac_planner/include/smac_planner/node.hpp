// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#ifndef SMAC_PLANNER__NODE_HPP_
#define SMAC_PLANNER__NODE_HPP_

#include <vector>
#include <iostream>
#include <queue>
#include <limits>

#include "smac_planner/constants.hpp"

namespace smac_planner
{

/**
 * @class smac_planner::Node
 * @brief Node implementation for graph
 */
class Node
{
public:
  /**
   * @brief A constructor for smac_planner::Node
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
   */
  explicit Node(unsigned char & cost_in, const unsigned int index)
  : last_node(nullptr),
    _cell_cost(static_cast<float>(cost_in)),
    _accumulated_cost(std::numeric_limits<float>::max()),
    _index(index),
    _was_visited(false),
    _is_queued(false)
  {
  }

  /**
   * @brief A destructor for smac_planner::Node
   */
  ~Node()
  {
    last_node = nullptr;
  }

  /**
   * @brief operator== for comparisons
   * @param Node right hand side node reference
   * @return If cell indicies are equal
   */
  bool operator==(const Node & rhs)
  {
    return this->_index == rhs._index;
  }

  /**
   * @brief Reset method for new search
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
   */
  void reset(const unsigned char & cost, const unsigned int index)
  {
    last_node = nullptr;
    _cell_cost = static_cast<float>(cost);
    _accumulated_cost = std::numeric_limits<float>::max();
    _index = index;
    _was_visited = false;
    _is_queued = false;
  }

  /**
   * @brief Gets the accumulated cost at this node
   * @return accumulated cost
   */
  float & getAccumulatedCost()
  {
    return _accumulated_cost;
  }

  /**
   * @brief Sets the accumulated cost at this node
   * @param reference to accumulated cost
   */
  void setAccumulatedCost(const float cost_in)
  {
    _accumulated_cost = cost_in;
  }

  /**
   * @brief Gets the costmap cost at this node
   * @return costmap cost
   */
  float & getCost()
  {
    return _cell_cost;
  }

  /**
   * @brief Gets if cell has been visited in search
   * @param If cell was visited
   */
  bool & wasVisited()
  {
    return _was_visited;
  }

  /**
   * @brief Sets if cell has been visited in search
   */
  void visited()
  {
    _was_visited = true;
    _is_queued = false;
  }

  /**
   * @brief Gets if cell is currently queued in search
   * @param If cell was queued
   */
  bool & isQueued()
  {
    return _is_queued;
  }

  /**
   * @brief Sets if cell is currently queued in search
   */
  void queued()
  {
    _is_queued = true;
  }

  /**
   * @brief Gets cell index
   * @return Reference to cell index
   */
  unsigned int & getIndex()
  {
    return _index;
  }

  /**
   * @brief Gets neighbors from node 
   * @param lookup_table The table with index offsets to find neighbors
   * @param neighbors The neighbors array to be filled
   * @param graph The templated graph used to retrieve the neighbors
   */
  void getNeighbors(
    std::vector<Node *> & neighbors,
    std::vector<Node> * graph,
    const bool & traverse_unknown)
  {
    // NOTE(stevemacenski): Irritatingly, the order here matters. If you start in free
    // space and then expand 8-connected, the first set of neighbors will be all cost
    // _neutral_cost. Then its expansion will all be 2 * _neutral_cost but now multiple
    // nodes are touching that node so the last cell to update the back pointer wins.
    // Thusly, the ordering ends with the cardinal directions for both sets such that
    // behavior is consistent in large free spaces between them.
    // 100  50   0
    // 100  50  50
    // 100 100 100   where lower-middle '100' is visited with same cost by both bottom '50' nodes
    // Therefore, it is valuable to have some low-potential across the entire map
    // rather than a small inflation around the obstacles
    int index;

    for (unsigned int i = 0; i != neighborhood_vec.size(); i++) {
      index = _index + neighborhood_vec[i];
      if (index > 0 && isNodeValid(traverse_unknown))
      {
        neighbors.push_back(& graph->operator[](index));
      }
    }
  }

  /**
   * @brief Check if this node is valid
   * @param traverse_unknown If we can explore unknown nodes on the graph
   * @return whether this node is valid and collision free
   */
  bool isNodeValid(const bool & traverse_unknown) {
    // NOTE(stevemacenski): Right now, we do not check if the node has wrapped around
    // the regular grid (e.g. your node is on the edge of the costmap and i+1
    // goes to the other side). This check would add compute time and my assertion is
    // that if you do wrap around, the heuristic will be so high it'll be added far
    // in the queue that it will never be called if a valid path exists.
    // This is intentionally un-included to increase speed, but be aware. If this causes
    // trouble, please file a ticket and we can address it then.

    // occupied node
    auto & cost = this->getCost();
    if (cost == OCCUPIED || cost == INSCRIBED) {
      return false;
    }

    // unknown node
    if (cost == UNKNOWN && ! traverse_unknown) {
      return false;
    }

    return true;
  }

  /**
   * @brief Initialize possible neighborhoods for 2D Nodes:
   * VON_NEUMANN --> 4-connected grid
   * MOORE       --> 8-connected grid
   * @param x_size The width of the underlying grid, in number of cells.
   * @param neighborhood The neighborhood type to assign to the nodes.
   */
  static void initNeighborhoods(const int & x_size, const Neighborhood & neighborhood) {
    switch (neighborhood) {
      case Neighborhood::UNKNOWN:
        throw std::runtime_error("Unknown neighborhood type selected.");
      case Neighborhood::VON_NEUMANN:
        neighborhood_vec = {-1, +1, -x_size, +x_size};
        break;
      case Neighborhood::MOORE:
        neighborhood_vec = {-1, +1, -x_size, +x_size, -x_size - 1,
                            -x_size + 1, +x_size - 1, +x_size + 1};
        break;
      default:
        throw std::runtime_error("Invalid neighborhood type selected.");
    }
  }

  Node * last_node;
  static std::vector<int> neighborhood_vec;


private:
  float _cell_cost;
  float _accumulated_cost;
  unsigned int _index;
  bool _was_visited;
  bool _is_queued;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__NODE_HPP_
