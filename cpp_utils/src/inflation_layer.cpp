/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include "map_loader/inflation_layer.hpp"

#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>
#include <iostream>

#include "yaml-cpp/yaml.h"
#include "map_loader/map_io.hpp"

using namespace nav2_map_server;

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_costmap_2d
{

InflationLayer::InflationLayer(Costmap2D * costmap, const std::string & yaml_filename)
: inflation_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  inflate_unknown_(false),
  inflate_around_unknown_(false),
  cell_inflation_radius_(0),
  cached_cell_inflation_radius_(0),
  resolution_(0),
  cache_length_(0),
  costmap_(costmap),
  yaml_filename_(yaml_filename)
{}

InflationLayer::~InflationLayer()
{}

void
InflationLayer::onInitialize()
{
  YAML::Node doc = YAML::LoadFile(expand_user_home_dir_if_needed(yaml_filename_, get_home_dir()));

  YAML::Node inflation_layer_node = doc["inflation_layer"];
  if (!inflation_layer_node) {
    throw std::runtime_error("Failed to find 'inflation_layer' node in YAML file");
  }

  inscribed_radius_ = yaml_get_value<double>(inflation_layer_node, "robot_radius");
  inflation_radius_ = yaml_get_value<double>(inflation_layer_node, "inflation_radius");
  cost_scaling_factor_ = yaml_get_value<double>(inflation_layer_node, "cost_scaling_factor");
  inflate_unknown_ = yaml_get_value<bool>(inflation_layer_node, "inflate_unknown");
  inflate_around_unknown_ = yaml_get_value<bool>(inflation_layer_node, "inflate_around_unknown");

  //debug
  // std::cout << "[Inflation] inscribed_radius_: " << inscribed_radius_ << std::endl;
  // std::cout << "[Inflation] inflation_radius_: " << inflation_radius_ << std::endl;
  // std::cout << "[Inflation] cost_scaling_factor_: " << cost_scaling_factor_ << std::endl;

  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  // cell_inflation_radius_ = cellDistance(inflation_radius_);
  matchSize();
}

void
InflationLayer::matchSize()
{
  resolution_ = costmap_->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  seen_ = std::vector<bool>(costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY(), false);
}


void
InflationLayer::updateCosts()
{

  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  for (auto & dist : inflation_cells_) {
    if (!dist.empty()) {
      std::cout << "[ERROR] The inflation list must be empty at the beginning of inflation" << std::endl;
    }
  }

  unsigned char * master_array = costmap_->getCharMap();
  unsigned int size_x = costmap_->getSizeInCellsX(), size_y = costmap_->getSizeInCellsY();

  if (seen_.size() != size_x * size_y) {
    std::cout << "[WARN] InflationLayer::updateCosts(): seen_ vector size is wrong" << std::endl;
    seen_ = std::vector<bool>(size_x * size_y, false);
  }

  std::fill(begin(seen_), end(seen_), false);

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  const int base_min_i = 0;
  const int base_min_j = 0;
  const int base_max_i = costmap_->getSizeInCellsX();
  const int base_max_j = costmap_->getSizeInCellsY();

  // Inflation list; we append cells to visit in a list associated with
  // its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before,
  // with a notable performance boost

  // Start with lethal obstacles: by definition distance is 0.0
  auto & obs_bin = inflation_cells_[0];
  for (int j = base_min_j; j < base_max_j; j++) {
    for (int i = base_min_i; i < base_max_i; i++) {
      int index = static_cast<int>(costmap_->getIndex(i, j));
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE || (inflate_around_unknown_ && cost == NO_INFORMATION)) {
        obs_bin.emplace_back(index, i, j, i, j);
      }
    }
  }

  // debug
  // std::cout << "[Inflation] obs_bin size: " << obs_bin.size() << std::endl;

  // Process cells by increasing distance; new cells are appended to the
  // corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  for (const auto & dist_bin : inflation_cells_) {
    for (std::size_t i = 0; i < dist_bin.size(); ++i) {
      // Do not use iterator or for-range based loops to
      // iterate though dist_bin, since it's size might
      // change when a new cell is enqueued, invalidating all iterators
      unsigned int index = dist_bin[i].index_;

      // ignore if already visited
      if (seen_[index]) {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = dist_bin[i].x_;
      unsigned int my = dist_bin[i].y_;
      unsigned int sx = dist_bin[i].src_x_;
      unsigned int sy = dist_bin[i].src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char cost = costLookup(mx, my, sx, sy);
      unsigned char old_cost = master_array[index];
      // In order to avoid artifacts appeared out of boundary areas
      // when some layer is going after inflation_layer,
      // we need to apply inflation_layer only to inside of given bounds
      if (static_cast<int>(mx) >= base_min_i &&
        static_cast<int>(my) >= base_min_j &&
        static_cast<int>(mx) < base_max_i &&
        static_cast<int>(my) < base_max_j)
      {
        if (old_cost == NO_INFORMATION &&
          (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        {
          master_array[index] = cost;
        } else {
          master_array[index] = std::max(old_cost, cost);
        }
      }

      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0) {
        enqueue(index - 1, mx - 1, my, sx, sy);
      }
      if (my > 0) {
        enqueue(index - size_x, mx, my - 1, sx, sy);
      }
      if (mx < size_x - 1) {
        enqueue(index + 1, mx + 1, my, sx, sy);
      }
      if (my < size_y - 1) {
        enqueue(index + size_x, mx, my + 1, sx, sy);
      }
    }
  }

  for (auto & dist : inflation_cells_) {
    dist.clear();
    dist.reserve(200);
  }

}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
void
InflationLayer::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index]) {
    // we compute our distance table one cell further than the
    // inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within
    // the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_) {
      return;
    }

    const unsigned int r = cell_inflation_radius_ + 2;

    // push the cell data onto the inflation list and mark
    inflation_cells_[distance_matrix_[mx - src_x + r][my - src_y + r]].emplace_back(
      index, mx, my, src_x, src_y);
  }
}

void
InflationLayer::computeCaches()
{
  if (cell_inflation_radius_ == 0) {
    return;
  }

  cache_length_ = cell_inflation_radius_ + 2;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);

    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i < cache_length_; ++i) {
    for (unsigned int j = 0; j < cache_length_; ++j) {
      cached_costs_[i * cache_length_ + j] = computeCost(cached_distances_[i * cache_length_ + j]);
    }
  }

  int max_dist = generateIntegerDistances();
  inflation_cells_.clear();
  inflation_cells_.resize(max_dist + 1);
  for (auto & dist : inflation_cells_) {
    dist.reserve(200);
  }

  // debug
  // std::cout << "[DEBUG] [InflationLayer::computeCaches]: cache_length_: " << cache_length_ << std::endl;
  // std::cout << "[DEBUG] [InflationLayer::max_dist]: " << max_dist << std::endl;
}

int
InflationLayer::generateIntegerDistances()
{
  const int r = cell_inflation_radius_ + 2;
  const int size = r * 2 + 1;

  std::vector<std::pair<int, int>> points;

  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      if (x * x + y * y <= r * r) {
        points.emplace_back(x, y);
      }
    }
  }

  std::sort(
    points.begin(), points.end(),
    [](const std::pair<int, int> & a, const std::pair<int, int> & b) -> bool {
      return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
    }
  );

  std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
  std::pair<int, int> last = {0, 0};
  int level = 0;
  for (auto const & p : points) {
    if (p.first * p.first + p.second * p.second !=
      last.first * last.first + last.second * last.second)
    {
      level++;
    }
    distance_matrix[p.first + r][p.second + r] = level;
    last = p;
  }

  distance_matrix_ = distance_matrix;
  return level;
}

}  // namespace nav2_costmap_2d
