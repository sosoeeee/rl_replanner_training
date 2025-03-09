/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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

#include "map_loader/static_layer.hpp"

#include <algorithm>
#include <string>

#include "yaml-cpp/yaml.h"
#include "map_loader/map_io.hpp"

using namespace nav2_map_server;

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_costmap_2d
{

StaticLayer::StaticLayer(int8_t * const & raw_data, const std::string & yaml_filename)
:raw_data_(raw_data), yaml_filename_(yaml_filename)
{
}

StaticLayer::~StaticLayer()
{
}

void
StaticLayer::onInitialize()
{
  getParameters();
}

void
StaticLayer::getParameters()
{
  YAML::Node doc = YAML::LoadFile(expand_user_home_dir_if_needed(yaml_filename_, get_home_dir()));

  YAML::Node static_layer_node = doc["static_layer"];
  if (!static_layer_node) {
    throw std::runtime_error("Failed to find 'static_layer' node in YAML file");
  }

  track_unknown_space_ = yaml_get_value<bool>(static_layer_node, "track_unknown_space");
  lethal_threshold_ = yaml_get_value<int>(static_layer_node, "lethal_threshold");
  unknown_cost_value_ = yaml_get_value<int>(static_layer_node, "unknown_cost_value");
  trinary_costmap_ = yaml_get_value<bool>(static_layer_node, "trinary_costmap");
  use_maximum_ = yaml_get_value<bool>(static_layer_node, "use_maximum");
}

unsigned char
StaticLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_) {
    return NO_INFORMATION;
  } else if (!track_unknown_space_ && value == unknown_cost_value_) {
    return FREE_SPACE;
  } else if (value >= lethal_threshold_) {
    return LETHAL_OBSTACLE;
  } else if (trinary_costmap_) {
    return FREE_SPACE;
  }

  double scale = static_cast<double>(value) / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void
StaticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D * master_grid)
{ 
  auto size_x = master_grid->getSizeInCellsX();
  auto size_y = master_grid->getSizeInCellsY();
  
  // load the map from raw data
  unsigned int index = 0;
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned char value = raw_data_[index];
      master_grid->setCost(j, i, interpretValue(value));
      ++index;
    }
  }
}

}  // namespace nav2_costmap_2d
