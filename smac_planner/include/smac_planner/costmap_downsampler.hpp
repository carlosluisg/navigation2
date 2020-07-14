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
  : _costmap(nullptr),
    _downsampled_costmap(nullptr)
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
   * @param downsampling_factor Multiplier for the costmap sresolution
   * @return A shared ptr to the downsampled costmap
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2D> initialize(
    nav2_costmap_2d::Costmap2D * costmap,
    const unsigned int & downsampling_factor)
  {
    _costmap = costmap;
    _downsampling_factor = downsampling_factor;
    updateCostmapSize();

    _downsampled_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>
      (_downsampled_size_x, _downsampled_size_y, _downsampled_resolution,
       _costmap->getOriginX(), _costmap->getOriginY(), UNKNOWN);

    return _downsampled_costmap;
  }

  /**
   * @brief Downsample the given costmap by the downsampling factor
   * @return A ptr to the downsampled costmap
   */
  nav2_costmap_2d::Costmap2D * downsample()
  {
    unsigned int new_mx, new_my;
    updateCostmapSize();

    // Adjust costmap size if needed
    if (_downsampled_costmap->getSizeInCellsX() != _downsampled_size_x ||
      _downsampled_costmap->getSizeInCellsY() != _downsampled_size_y) {
      resizeCostmap();
    }

    // Assign costs
    for (int i = 0; i < _downsampled_size_x * _downsampled_size_y; ++i) {
      new_mx = i % _downsampled_size_x;
      new_my = i / _downsampled_size_x;
      setCostOfCell(new_mx, new_my);
    }

    return _downsampled_costmap.get();
  }

private:
  /**
   * Update the sizes X-Y of the costmap and its downsampled version
   */
  void updateCostmapSize()
  {
    _size_x = _costmap->getSizeInCellsX();
    _size_y = _costmap->getSizeInCellsY();
    _downsampled_size_x = ceil(static_cast<float>(_size_x) / _downsampling_factor);
    _downsampled_size_y = ceil(static_cast<float>(_size_y) / _downsampling_factor);
    _downsampled_resolution = _downsampling_factor * _costmap->getResolution();
  }

  /**
   * Resize the downsampled costmap. Used in case the costmap changes and we need to update the downsampled version
   */
  void resizeCostmap()
  {
    _downsampled_costmap->resizeMap(
      _downsampled_size_x,
      _downsampled_size_y,
      _downsampled_resolution,
      _costmap->getOriginX(),
      _costmap->getOriginY());
  }

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

    for(int j = 0; j < _downsampling_factor * _downsampling_factor; ++j) {
      mx = std::min(x_offset + j % _downsampling_factor, _size_x - 1);
      my = std::min(y_offset + j / _downsampling_factor, _size_y - 1);
      cost = std::max(cost, _costmap->getCost(mx, my));
    }
    _downsampled_costmap->setCost(new_mx, new_my, cost);
  }

  unsigned int _size_x;
  unsigned int _size_y;
  unsigned int _downsampled_size_x;
  unsigned int _downsampled_size_y;
  unsigned int _downsampling_factor;
  float _downsampled_resolution;
  nav2_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> _downsampled_costmap;
};

}  // namespace smac_planner

#endif // SMAC_PLANNER__DOWNSAMPLER_HPP_
