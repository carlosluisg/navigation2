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
  : _sampling_factor(0)
  {
  }

  /**
   * @brief A destructor for CostmapDownsampler
   */
  ~CostmapDownsampler()
  {
  }

  /**
   * @brief Downsample the given costmap to the closest approximation of the target resolution
   * @param original_costmap The costmap we want to downsample
   * @param target_resolution The desired size of a cell in the new costmap, in meters
   * @return A pointer to the newly created downsampled costmap
   * Note: despite returning a pointer, this class keeps ownership of the resource it created.
   */
  nav2_costmap_2d::Costmap2D * downsample(
    nav2_costmap_2d::Costmap2D * original_costmap,
    const float & target_resolution)
  {
    _original_size_x = original_costmap->getSizeInCellsX();
    _original_size_y = original_costmap->getSizeInCellsY();
    _original_costmap = original_costmap;
    _sampling_factor = round(target_resolution / original_costmap->getResolution());
    float rounded_resolution = _sampling_factor * original_costmap->getResolution();
    unsigned int map_cells_size_x = ceil(static_cast<float>(original_costmap->getSizeInCellsX()) / _sampling_factor);
    unsigned int map_cells_size_y = ceil(static_cast<float>(original_costmap->getSizeInCellsY()) / _sampling_factor);

    _downsampled_costmap = std::make_unique<nav2_costmap_2d::Costmap2D>
      (map_cells_size_x, map_cells_size_y, rounded_resolution,
       original_costmap->getOriginX(), original_costmap->getOriginY(), UNKNOWN);

    setCosts();
    return _downsampled_costmap.get();
  }

private:
  /**
   * @brief Iterate through every cell of the downsampled costmap and assign a cost to it
   */
  void setCosts()
  {
    unsigned int new_size_x = _downsampled_costmap->getSizeInCellsX();
    unsigned int new_size_y = _downsampled_costmap->getSizeInCellsY();

    for (int i = 0; i < new_size_x * new_size_y; ++i) {
      int new_mx = i % new_size_x;
      int new_my = i / new_size_x;
      setCostOfCell(new_mx, new_my);
    }
  }

  /**
   * @brief Explore all subcells of the original costmap and assign the max cost to the new (downsampled) cell
   * @param new_mx The X-coordinate of the cell in the new costmap
   * @param new_my The Y-coordinate of the cell in the new costmap
   */
  void setCostOfCell(
    const unsigned int & new_mx, const unsigned int & new_my)
  {
    unsigned char cost = 0;
    for(int j = 0; j < _sampling_factor * _sampling_factor; ++j)
    {
      unsigned int mx = std::min(new_mx * _sampling_factor + j % _sampling_factor, _original_size_x - 1);
      unsigned int my = std::min(new_my * _sampling_factor + j / _sampling_factor, _original_size_y - 1);
      cost = std::max(cost, _original_costmap->getCost(mx, my));
    }
    _downsampled_costmap->setCost(new_mx, new_my, cost);
  }

  unsigned int _sampling_factor;
  unsigned int _original_size_x;
  unsigned int _original_size_y;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> _downsampled_costmap;
  nav2_costmap_2d::Costmap2D * _original_costmap;
};

}  // namespace smac_planner

#endif // SMAC_PLANNER__DOWNSAMPLER_HPP_
