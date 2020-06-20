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

#ifndef SMAC_PLANNER__TYPES_HPP_
#define SMAC_PLANNER__TYPES_HPP_

#include <vector>
#include <unordered_map>
#include <utility>
#include <queue>

#include "smac_planner/node.hpp"

namespace smac_planner
{
typedef std::vector<unsigned int> IndexPath;

typedef std::pair<float, float> Coordinates;

typedef std::pair<double, double> DoubleCoordinates;

typedef Node *  NodePtr;

typedef std::vector<Node> Graph;

typedef std::vector<Node *> NodeVector;

typedef std::pair<float, Node *> NodeElement;

typedef std::pair<float, unsigned int> NodeHeuristicPair;

struct NodeComparator
{
  bool operator()(const NodeElement & a, const NodeElement & b) const
  {
    return a.first > b.first;
  }
};

typedef std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator> NodeQueue;
}  // namespace smac_planner

#endif  // SMAC_PLANNER__TYPES_HPP_
