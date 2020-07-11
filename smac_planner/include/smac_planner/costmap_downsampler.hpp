// Copyright (c) 2020, Carlos Luis
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

#ifndef SMAC_PLANNER__DOWNSAMPLER_HPP_
#define SMAC_PLANNER__DOWNSAMPLER_HPP_

#include <algorithm>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "smac_planner/constants.hpp"

namespace smac_planner
{

/**
 * @class smac_planner::CostmapDownsampler
 * @brief A costmap downsampler for more efficient path planning
 */
class CostmapDownsampler
{
public:
  /**
   * @brief A constructor for CostmapDownsampler
   */
  explicit CostmapDownsampler()
  : _downsampling_factor(0)
  {
  }

  /**
   * @brief A destructor for CostmapDownsampler
   */
  ~CostmapDownsampler()
  {
  }

  /**
   * @brief Initialize the downsampled costmap object
   * @param costmap The costmap we want to downsample
   * @param downsampling_factor Multiplier for the costmap resolution
   * @return A shared ptr to the downsampled costmap
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2D> initialize(
    nav2_costmap_2d::Costmap2D * costmap,
    const unsigned int & downsampling_factor)
  {
    _size_x = costmap->getSizeInCellsX();
    _size_y = costmap->getSizeInCellsY();
    _costmap = costmap;
    _downsampling_factor = downsampling_factor;
    float rounded_resolution = _downsampling_factor * costmap->getResolution();
    unsigned int map_cells_size_x = ceil(static_cast<float>(_size_x) / _downsampling_factor);
    unsigned int map_cells_size_y = ceil(static_cast<float>(_size_y) / _downsampling_factor);

    _downsampled_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>
            (map_cells_size_x, map_cells_size_y, rounded_resolution,
             _costmap->getOriginX(), _costmap->getOriginY(), UNKNOWN);
    return _downsampled_costmap;
  }

  /**
   * @brief Downsample the given costmap by the downsampling factor
   */
  void downsample()
  {
    unsigned int new_size_x = _downsampled_costmap->getSizeInCellsX();
    unsigned int new_size_y = _downsampled_costmap->getSizeInCellsY();
    unsigned int new_mx, new_my;

    for (int i = 0; i < new_size_x * new_size_y; ++i) {
      new_mx = i % new_size_x;
      new_my = i / new_size_x;
      setCostOfCell(new_mx, new_my);
    }
  }

private:

  /**
   * @brief Explore all subcells of the original costmap and assign the max cost to the new (downsampled) cell
   * @param new_mx The X-coordinate of the cell in the new costmap
   * @param new_my The Y-coordinate of the cell in the new costmap
   */
  void setCostOfCell(
    const unsigned int & new_mx,
    const unsigned int & new_my)
  {
    unsigned int mx, my;
    unsigned char cost = 0;
    unsigned int x_offset = new_mx * _downsampling_factor;
    unsigned int y_offset = new_my * _downsampling_factor;

    for(int j = 0; j < _downsampling_factor * _downsampling_factor; ++j)
    {
      mx = std::min(x_offset + j % _downsampling_factor, _size_x - 1);
      my = std::min(y_offset + j / _downsampling_factor, _size_y - 1);
      cost = std::max(cost, _costmap->getCost(mx, my));
    }
    _downsampled_costmap->setCost(new_mx, new_my, cost);
  }

  unsigned int _downsampling_factor;
  unsigned int _size_x;
  unsigned int _size_y;
  nav2_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> _downsampled_costmap;
};

}  // namespace smac_planner

#endif // SMAC_PLANNER__DOWNSAMPLER_HPP_
