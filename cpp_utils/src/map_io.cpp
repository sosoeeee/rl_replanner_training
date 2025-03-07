/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "map_loader/map_io.hpp"

#ifndef _WIN32
#include <libgen.h>
#endif
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <cstdlib>

#include "Magick++.h"
#include "yaml-cpp/yaml.h"
// #include "tf2/LinearMath/Matrix3x3.h"
// #include "tf2/LinearMath/Quaternion.h"
#include "map_loader/occ_grid_values.hpp"


namespace nav2_map_server
{
// === Map input part ===

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
/// including the name of the failed key
/// @throw YAML::Exception
template<typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

std::string get_home_dir()
{
  if (const char * home_dir = std::getenv("HOME")) {
    return std::string{home_dir};
  }
  return std::string{};
}

std::string expand_user_home_dir_if_needed(
  std::string yaml_filename,
  std::string home_variable_value)
{
  if (yaml_filename.size() < 2 || !(yaml_filename[0] == '~' && yaml_filename[1] == '/')) {
    return yaml_filename;
  }
  if (home_variable_value.empty()) {
    std::cout << "[INFO] [map_io]: Map yaml file name starts with '~/' but no HOME variable set. \n"
              << "[INFO] [map_io] User home dir will be not expanded \n";
    return yaml_filename;
  }
  const std::string prefix{home_variable_value};
  return yaml_filename.replace(0, 1, prefix);
}

LoadParameters loadMapYaml(const std::string & yaml_filename)
{
  YAML::Node doc = YAML::LoadFile(expand_user_home_dir_if_needed(yaml_filename, get_home_dir()));
  LoadParameters load_parameters;

  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }
  if (image_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }
  load_parameters.image_file_name = image_file_name;

  load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
  load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
  if (load_parameters.origin.size() != 3) {
    throw YAML::Exception(
            doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
            std::to_string(load_parameters.origin.size()));
  }

  load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  load_parameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    load_parameters.mode = MapMode::Trinary;
  } else {
    load_parameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
  }

  try {
    load_parameters.negate = yaml_get_value<int>(doc, "negate");
  } catch (YAML::Exception &) {
    load_parameters.negate = yaml_get_value<bool>(doc, "negate");
  }

  std::cout << "[DEBUG] [map_io]: resolution: " << load_parameters.resolution << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[0]: " << load_parameters.origin[0] << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[1]: " << load_parameters.origin[1] << std::endl;
  std::cout << "[DEBUG] [map_io]: origin[2]: " << load_parameters.origin[2] << std::endl;
  std::cout << "[DEBUG] [map_io]: free_thresh: " << load_parameters.free_thresh << std::endl;
  std::cout << "[DEBUG] [map_io]: occupied_thresh: " << load_parameters.occupied_thresh <<
    std::endl;
  std::cout << "[DEBUG] [map_io]: mode: " << map_mode_to_string(load_parameters.mode) << std::endl;
  std::cout << "[DEBUG] [map_io]: negate: " << load_parameters.negate << std::endl;  //NOLINT

  return load_parameters;
}

void loadMapFromFile(
  const LoadParameters & load_parameters)
{
  Magick::InitializeMagick(nullptr);
  // nav_msgs::msg::OccupancyGrid msg;

  std::cout << "[INFO] [map_io]: Loading image_file: " <<
    load_parameters.image_file_name << std::endl;
  Magick::Image img(load_parameters.image_file_name);

  // // Copy the image data into the map structure
  // msg.info.width = img.size().width();
  // msg.info.height = img.size().height();

  // msg.info.resolution = load_parameters.resolution;
  // msg.info.origin.position.x = load_parameters.origin[0];
  // msg.info.origin.position.y = load_parameters.origin[1];
  // msg.info.origin.position.z = 0.0;
  // msg.info.origin.orientation = orientationAroundZAxis(load_parameters.origin[2]);

  // // Allocate space to hold the data
  // msg.data.resize(msg.info.width * msg.info.height);

  // // Copy pixel data into the map structure
  // for (size_t y = 0; y < msg.info.height; y++) {
  //   for (size_t x = 0; x < msg.info.width; x++) {
  //     auto pixel = img.pixelColor(x, y);

  //     std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(),
  //       pixel.blueQuantum()};
  //     if (load_parameters.mode == MapMode::Trinary && img.matte()) {
  //       // To preserve existing behavior, average in alpha with color channels in Trinary mode.
  //       // CAREFUL. alpha is inverted from what you might expect. High = transparent, low = opaque
  //       channels.push_back(MaxRGB - pixel.alphaQuantum());
  //     }
  //     double sum = 0;
  //     for (auto c : channels) {
  //       sum += c;
  //     }
  //     /// on a scale from 0.0 to 1.0 how bright is the pixel?
  //     double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());

  //     // If negate is true, we consider blacker pixels free, and whiter
  //     // pixels occupied. Otherwise, it's vice versa.
  //     /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
  //     double occ = (load_parameters.negate ? shade : 1.0 - shade);

  //     int8_t map_cell;
  //     switch (load_parameters.mode) {
  //       case MapMode::Trinary:
  //         if (load_parameters.occupied_thresh < occ) {
  //           map_cell = nav2_util::OCC_GRID_OCCUPIED;
  //         } else if (occ < load_parameters.free_thresh) {
  //           map_cell = nav2_util::OCC_GRID_FREE;
  //         } else {
  //           map_cell = nav2_util::OCC_GRID_UNKNOWN;
  //         }
  //         break;
  //       case MapMode::Scale:
  //         if (pixel.alphaQuantum() != OpaqueOpacity) {
  //           map_cell = nav2_util::OCC_GRID_UNKNOWN;
  //         } else if (load_parameters.occupied_thresh < occ) {
  //           map_cell = nav2_util::OCC_GRID_OCCUPIED;
  //         } else if (occ < load_parameters.free_thresh) {
  //           map_cell = nav2_util::OCC_GRID_FREE;
  //         } else {
  //           map_cell = std::rint(
  //             (occ - load_parameters.free_thresh) /
  //             (load_parameters.occupied_thresh - load_parameters.free_thresh) * 100.0);
  //         }
  //         break;
  //       case MapMode::Raw: {
  //           double occ_percent = std::round(shade * 255);
  //           if (nav2_util::OCC_GRID_FREE <= occ_percent &&
  //             occ_percent <= nav2_util::OCC_GRID_OCCUPIED)
  //           {
  //             map_cell = static_cast<int8_t>(occ_percent);
  //           } else {
  //             map_cell = nav2_util::OCC_GRID_UNKNOWN;
  //           }
  //           break;
  //         }
  //       default:
  //         throw std::runtime_error("Invalid map mode");
  //     }
  //     msg.data[msg.info.width * (msg.info.height - y - 1) + x] = map_cell;
  //   }
  // }

  // // Since loadMapFromFile() does not belong to any node, publishing in a system time.
  // rclcpp::Clock clock(RCL_SYSTEM_TIME);
  // msg.info.map_load_time = clock.now();
  // msg.header.frame_id = "map";
  // msg.header.stamp = clock.now();

  // std::cout <<
  //   "[DEBUG] [map_io]: Read map " << load_parameters.image_file_name << ": " << msg.info.width <<
  //   " X " << msg.info.height << " map @ " << msg.info.resolution << " m/cell" << std::endl;

  // map = msg;
}

LOAD_MAP_STATUS loadMapFromYaml(
  const std::string & yaml_file)
{
  if (yaml_file.empty()) {
    std::cerr << "[ERROR] [map_io]: YAML file name is empty, can't load!" << std::endl;
    return MAP_DOES_NOT_EXIST;
  }
  std::cout << "[INFO] [map_io]: Loading yaml file: " << yaml_file << std::endl;
  LoadParameters load_parameters;
  try {
    load_parameters = loadMapYaml(yaml_file);
  } catch (YAML::Exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed processing YAML file " << yaml_file << " at position (" <<
      e.mark.line << ":" << e.mark.column << ") for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed to parse map YAML loaded from file " << yaml_file <<
      " for reason: " << e.what() << std::endl;
    return INVALID_MAP_METADATA;
  }
  try {
    loadMapFromFile(load_parameters);
  } catch (std::exception & e) {
    std::cerr <<
      "[ERROR] [map_io]: Failed to load image file " << load_parameters.image_file_name <<
      " for reason: " << e.what() << std::endl;
    return INVALID_MAP_DATA;
  }

  return LOAD_MAP_SUCCESS;
}

}  // namespace nav2_map_server
