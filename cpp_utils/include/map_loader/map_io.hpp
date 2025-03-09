// Copyright (c) 2018 Intel Corporation
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
// limitations under the License.

/* OccupancyGrid map input-output library */

#ifndef NAV2_MAP_SERVER__MAP_IO_HPP_
#define NAV2_MAP_SERVER__MAP_IO_HPP_

#include <string>
#include <vector>

#include "map_mode.hpp"
#include "yaml-cpp/yaml.h"

/* Map input part */

namespace nav2_map_server
{

template<typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key);

std::string get_home_dir();

std::string expand_user_home_dir_if_needed(
  std::string yaml_filename,
  std::string home_variable_value);

struct LoadParameters
{
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};

typedef enum
{
  LOAD_MAP_SUCCESS,
  MAP_DOES_NOT_EXIST,
  INVALID_MAP_METADATA,
  INVALID_MAP_DATA
} LOAD_MAP_STATUS;

/**
 * @brief Load and parse the given YAML file
 * @param yaml_filename Name of the map file passed though parameter
 * @return Map loading parameters obtained from YAML file
 * @throw YAML::Exception
 */
LoadParameters loadMapYaml(const std::string & yaml_filename);

/**
 * @brief Load the image from map file and generate an OccupancyGrid
 * @param load_parameters Parameters of loading map
 * @throw std::exception
 */
void loadMapFromFile(
  const LoadParameters & load_parameters,
  unsigned int & size_x,
  unsigned int & size_y,
  double & resolution,
  double & origin_x,
  double & origin_y,
  int8_t * & data);

/**
 * @brief Load the map YAML, image from map file and
 * generate an OccupancyGrid
 * @param yaml_file Name of input YAML file
 * @param map Output loaded map
 * @return status of map loaded
 */
LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file,
  unsigned int & size_x,
  unsigned int & size_y,
  double & resolution,
  double & origin_x,
  double & origin_y,
  int8_t * & data);


/* Map output part */

struct SaveParameters
{
  std::string map_file_name{""};
  std::string image_format{""};
  double free_thresh{0.0};
  double occupied_thresh{0.0};
  MapMode mode{MapMode::Trinary};
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_IO_HPP_
