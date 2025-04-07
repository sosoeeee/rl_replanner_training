/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include "teb_local_planner/teb_config.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include "teb_local_planner/logger.h" 

namespace teb_local_planner
{

template <typename T>
T TebConfig::getYamlValue(const YAML::Node& node, const std::string& key, const T& default_value) const 
{
  if (node[key]) {
    return node[key].as<T>();
  }
  return default_value;
}

void TebConfig::loadParamsFromYaml(const std::string & yaml_filename)
{
  try {
    YAML::Node config = YAML::LoadFile(yaml_filename);
    
    YAML::Node params = config;
    
    // trajectory
    trajectory.teb_autosize = getYamlValue<double>(params, "teb_autosize", trajectory.teb_autosize);
    trajectory.dt_ref = getYamlValue<double>(params, "dt_ref", trajectory.dt_ref);
    trajectory.dt_hysteresis = getYamlValue<double>(params, "dt_hysteresis", trajectory.dt_hysteresis);
    trajectory.max_samples = getYamlValue<int>(params, "max_samples", trajectory.max_samples);
    trajectory.global_plan_overwrite_orientation = getYamlValue<bool>(params, "global_plan_overwrite_orientation", trajectory.global_plan_overwrite_orientation);
    trajectory.allow_init_with_backwards_motion = getYamlValue<bool>(params, "allow_init_with_backwards_motion", trajectory.allow_init_with_backwards_motion);
    trajectory.exact_arc_length = getYamlValue<bool>(params, "exact_arc_length", trajectory.exact_arc_length);
    
    // robot
    robot.max_vel_x = getYamlValue<double>(params, "max_vel_x", robot.max_vel_x);
    robot.max_vel_y = getYamlValue<double>(params, "max_vel_y", robot.max_vel_y);
    robot.max_vel_theta = getYamlValue<double>(params, "max_vel_theta", robot.max_vel_theta);
    robot.acc_lim_x = getYamlValue<double>(params, "acc_lim_x", robot.acc_lim_x);
    robot.acc_lim_y = getYamlValue<double>(params, "acc_lim_y", robot.acc_lim_y);
    robot.acc_lim_theta = getYamlValue<double>(params, "acc_lim_theta", robot.acc_lim_theta);
    
    if (params["footprint_model"]) {
      YAML::Node footprint_node = params["footprint_model"];
      model_name = getYamlValue<std::string>(footprint_node, "type", "point");
      
      if (model_name == "circular") {
        radius = getYamlValue<double>(footprint_node, "radius", 0.2);
        robot_model = std::make_shared<CircularRobotFootprint>(radius);
      } else if (model_name == "point") {
        robot_model = std::make_shared<PointRobotFootprint>();
      } else if (model_name == "line") {
        if (footprint_node["line_start"] && footprint_node["line_end"]) {
          line_start = footprint_node["line_start"].as<std::vector<double>>();
          line_end = footprint_node["line_end"].as<std::vector<double>>();
          if (line_start.size() == 2 && line_end.size() == 2) {
            robot_model = std::make_shared<LineRobotFootprint>(
              Eigen::Vector2d(line_start[0], line_start[1]), 
              Eigen::Vector2d(line_end[0], line_end[1]));
          } else {
            std::cerr << "Error: line_start and line_end must contain exactly 2 elements" << std::endl;
            robot_model = std::make_shared<PointRobotFootprint>();
          }
        }
      } else if (model_name == "two_circles") {
        front_offset = getYamlValue<double>(footprint_node, "front_offset", 0.2);
        front_radius = getYamlValue<double>(footprint_node, "front_radius", 0.2);
        rear_offset = getYamlValue<double>(footprint_node, "rear_offset", 0.2);
        rear_radius = getYamlValue<double>(footprint_node, "rear_radius", 0.2);
        robot_model = std::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
      } else if (model_name == "polygon") {
        footprint_string = getYamlValue<std::string>(footprint_node, "vertices", "");
        std::cerr << "Polygon footprint not fully implemented in loadParamsFromYaml" << std::endl;
        robot_model = std::make_shared<PointRobotFootprint>();
      } else {
        std::cerr << "Unknown footprint model: " << model_name << ". Using point model." << std::endl;
        robot_model = std::make_shared<PointRobotFootprint>();
      }
    }
    
    // GoalTolerance
    goal_tolerance.free_goal_vel = getYamlValue<bool>(params, "free_goal_vel", goal_tolerance.free_goal_vel);
    
    // Obstacles
    obstacles.min_obstacle_dist = getYamlValue<double>(params, "min_obstacle_dist", obstacles.min_obstacle_dist);
    obstacles.inflation_dist = getYamlValue<double>(params, "inflation_dist", obstacles.inflation_dist);
    obstacles.dynamic_obstacle_inflation_dist = getYamlValue<double>(params, "dynamic_obstacle_inflation_dist", obstacles.dynamic_obstacle_inflation_dist);
    obstacles.include_dynamic_obstacles = getYamlValue<bool>(params, "include_dynamic_obstacles", obstacles.include_dynamic_obstacles);
    obstacles.include_costmap_obstacles = getYamlValue<bool>(params, "include_costmap_obstacles", obstacles.include_costmap_obstacles);
    obstacles.costmap_obstacles_behind_robot_dist = getYamlValue<double>(params, "costmap_obstacles_behind_robot_dist", obstacles.costmap_obstacles_behind_robot_dist);
    obstacles.obstacle_poses_affected = getYamlValue<int>(params, "obstacle_poses_affected", obstacles.obstacle_poses_affected);
    obstacles.costmap_converter_plugin = getYamlValue<std::string>(params, "costmap_converter_plugin", obstacles.costmap_converter_plugin);
    obstacles.costmap_converter_spin_thread = getYamlValue<bool>(params, "costmap_converter_spin_thread", obstacles.costmap_converter_spin_thread);
    obstacles.costmap_converter_rate = getYamlValue<int>(params, "costmap_converter_rate", obstacles.costmap_converter_rate);
    
    // Optimization
    optim.no_inner_iterations = getYamlValue<int>(params, "no_inner_iterations", optim.no_inner_iterations);
    optim.no_outer_iterations = getYamlValue<int>(params, "no_outer_iterations", optim.no_outer_iterations);
    optim.optimization_activate = getYamlValue<bool>(params, "optimization_activate", optim.optimization_activate);
    optim.optimization_verbose = getYamlValue<bool>(params, "optimization_verbose", optim.optimization_verbose);
    optim.penalty_epsilon = getYamlValue<double>(params, "penalty_epsilon", optim.penalty_epsilon);
    optim.weight_max_vel_x = getYamlValue<double>(params, "weight_max_vel_x", optim.weight_max_vel_x);
    optim.weight_max_vel_y = getYamlValue<double>(params, "weight_max_vel_y", optim.weight_max_vel_y);
    optim.weight_max_vel_theta = getYamlValue<double>(params, "weight_max_vel_theta", optim.weight_max_vel_theta);
    optim.weight_acc_lim_x = getYamlValue<double>(params, "weight_acc_lim_x", optim.weight_acc_lim_x);
    optim.weight_acc_lim_y = getYamlValue<double>(params, "weight_acc_lim_y", optim.weight_acc_lim_y);
    optim.weight_acc_lim_theta = getYamlValue<double>(params, "weight_acc_lim_theta", optim.weight_acc_lim_theta);
    optim.weight_kinematics_nh = getYamlValue<double>(params, "weight_kinematics_nh", optim.weight_kinematics_nh);
    optim.weight_kinematics_forward_drive = getYamlValue<double>(params, "weight_kinematics_forward_drive", optim.weight_kinematics_forward_drive);
    optim.weight_kinematics_turning_radius = getYamlValue<double>(params, "weight_kinematics_turning_radius", optim.weight_kinematics_turning_radius);
    optim.weight_optimaltime = getYamlValue<double>(params, "weight_optimaltime", optim.weight_optimaltime);
    optim.weight_shortest_path = getYamlValue<double>(params, "weight_shortest_path", optim.weight_shortest_path);
    optim.weight_obstacle = getYamlValue<double>(params, "weight_obstacle", optim.weight_obstacle);
    optim.weight_inflation = getYamlValue<double>(params, "weight_inflation", optim.weight_inflation);
    optim.weight_dynamic_obstacle = getYamlValue<double>(params, "weight_dynamic_obstacle", optim.weight_dynamic_obstacle);
    optim.weight_dynamic_obstacle_inflation = getYamlValue<double>(params, "weight_dynamic_obstacle_inflation", optim.weight_dynamic_obstacle_inflation);
    optim.weight_viapoint = getYamlValue<double>(params, "weight_viapoint", optim.weight_viapoint);
    optim.weight_adapt_factor = getYamlValue<double>(params, "weight_adapt_factor", optim.weight_adapt_factor);
    optim.obstacle_cost_exponent = getYamlValue<double>(params, "obstacle_cost_exponent", optim.obstacle_cost_exponent);
    
    // HomotopyClasses
    hcp.enable_homotopy_class_planning = getYamlValue<bool>(params, "enable_homotopy_class_planning", hcp.enable_homotopy_class_planning);
    hcp.enable_multithreading = getYamlValue<bool>(params, "enable_multithreading", hcp.enable_multithreading);
    hcp.max_number_classes = getYamlValue<int>(params, "max_number_classes", hcp.max_number_classes);
    hcp.selection_cost_hysteresis = getYamlValue<double>(params, "selection_cost_hysteresis", hcp.selection_cost_hysteresis);
    hcp.selection_prefer_initial_plan = getYamlValue<double>(params, "selection_prefer_initial_plan", hcp.selection_prefer_initial_plan);
    hcp.selection_obst_cost_scale = getYamlValue<double>(params, "selection_obst_cost_scale", hcp.selection_obst_cost_scale);
    hcp.selection_alternative_time_cost = getYamlValue<bool>(params, "selection_alternative_time_cost", hcp.selection_alternative_time_cost);
    hcp.roadmap_graph_no_samples = getYamlValue<int>(params, "roadmap_graph_no_samples", hcp.roadmap_graph_no_samples);
    hcp.roadmap_graph_area_width = getYamlValue<double>(params, "roadmap_graph_area_width", hcp.roadmap_graph_area_width);
    hcp.roadmap_graph_area_length_scale = getYamlValue<double>(params, "roadmap_graph_area_length_scale", hcp.roadmap_graph_area_length_scale);
    hcp.h_signature_prescaler = getYamlValue<double>(params, "h_signature_prescaler", hcp.h_signature_prescaler);
    hcp.h_signature_threshold = getYamlValue<double>(params, "h_signature_threshold", hcp.h_signature_threshold);
    hcp.obstacle_heading_threshold = getYamlValue<double>(params, "obstacle_heading_threshold", hcp.obstacle_heading_threshold);
    hcp.switching_blocking_period = getYamlValue<double>(params, "switching_blocking_period", hcp.switching_blocking_period);
    hcp.viapoints_all_candidates = getYamlValue<bool>(params, "viapoints_all_candidates", hcp.viapoints_all_candidates);
    hcp.delete_detours_backwards = getYamlValue<bool>(params, "delete_detours_backwards", hcp.delete_detours_backwards);
    hcp.max_ratio_detours_duration_best_duration = getYamlValue<double>(params, "max_ratio_detours_duration_best_duration", hcp.max_ratio_detours_duration_best_duration);
    hcp.visualize_hc_graph = getYamlValue<bool>(params, "visualize_hc_graph", hcp.visualize_hc_graph);
    hcp.visualize_with_time_as_z_axis_scale = getYamlValue<double>(params, "visualize_with_time_as_z_axis_scale", hcp.visualize_with_time_as_z_axis_scale);
    
    // Recovery
    recovery.shrink_horizon_backup = getYamlValue<bool>(params, "shrink_horizon_backup", recovery.shrink_horizon_backup);
    recovery.shrink_horizon_min_duration = getYamlValue<double>(params, "shrink_horizon_min_duration", recovery.shrink_horizon_min_duration);
    recovery.oscillation_recovery = getYamlValue<bool>(params, "oscillation_recovery", recovery.oscillation_recovery);
    recovery.oscillation_v_eps = getYamlValue<double>(params, "oscillation_v_eps", recovery.oscillation_v_eps);
    recovery.oscillation_omega_eps = getYamlValue<double>(params, "oscillation_omega_eps", recovery.oscillation_omega_eps);
    recovery.oscillation_recovery_min_duration = getYamlValue<double>(params, "oscillation_recovery_min_duration", recovery.oscillation_recovery_min_duration);
    recovery.oscillation_filter_duration = getYamlValue<double>(params, "oscillation_filter_duration", recovery.oscillation_filter_duration);
    
    checkParameters();
    std::cout << "TEB parameters loaded from " << yaml_filename << std::endl;
    
  } catch (const YAML::Exception& e) {
    std::cerr << "Error loading yaml file: " << e.what() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
}

void TebConfig::checkParameters() const
{
  const std::string logger_name = "TEBLocalPlanner";
  
  // positive backward velocity?
  if (robot.max_vel_x_backwards <= 0)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  
  // bounds smaller than penalty epsilon
  if (robot.max_vel_x <= optim.penalty_epsilon)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_theta <= optim.penalty_epsilon)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_x <= optim.penalty_epsilon)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_theta <= optim.penalty_epsilon)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
      
  // dt_ref and dt_hyst
  if (trajectory.dt_ref <= trajectory.dt_hysteresis)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");
    
  // min number of samples
  if (trajectory.min_samples <3)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");
  
  // costmap obstacle behind robot
  if (obstacles.costmap_obstacles_behind_robot_dist < 0)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");
    
  // hcp: obstacle heading threshold
  if (hcp.obstacle_keypoint_offset>=1 || hcp.obstacle_keypoint_offset<=0)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");
  
  // carlike
  if (robot.cmd_angle_instead_rotvel && robot.wheelbase==0)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");
  
  if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius==0)
    LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");
  
  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0)
      LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");
  
  if (recovery.oscillation_filter_duration < 0)
      LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");
  
  if (optim.weight_optimaltime <= 0)
      LOGGER_WARN(logger_name, "TebLocalPlannerROS() Param Warning: parameter weight_optimaltime shoud be > 0 (even if weight_shortest_path is in use)");
}    

} // namespace teb_local_planner
