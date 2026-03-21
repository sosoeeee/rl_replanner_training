#include "path_planner/navfn_planner_with_cone.hpp"
#include "path_planner/intention_constraint.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "path_planner/navfn.hpp"
#include "yaml-cpp/yaml.h"
#include "map_loader/map_io.hpp"
#include "map_loader/cost_values.hpp"


namespace nav2_navfn_planner_with_cone
{

NavfnPlannerWithCone::NavfnPlannerWithCone()
: costmap_(nullptr), enabled_(false)
{
}

NavfnPlannerWithCone::~NavfnPlannerWithCone()
{
}

void
NavfnPlannerWithCone::configure(nav2_costmap_2d::Costmap2D * costmap, const std::string & yaml_filename)
{
  costmap_ = costmap;

  // // debug
  // std::cout << "[Path Planner] costmap size_x: " << costmap_->getSizeInCellsX() << ", size_y: " << costmap_->getSizeInCellsY() << std::endl;
  // std::cout << "[Path Planner] costmap resolution: " << costmap_->getResolution() << std::endl;
  // std::cout << "[Path Planner] costmap origin_x: " << costmap_->getOriginX() << ", origin_y: " << costmap_->getOriginY() << std::endl;

  resolution_ = costmap_->getResolution();

  YAML::Node doc = YAML::LoadFile(nav2_map_server::expand_user_home_dir_if_needed(yaml_filename, nav2_map_server::get_home_dir()));

  tolerance_ = nav2_map_server::yaml_get_value<double>(doc, "tolerance");
  use_astar_ = nav2_map_server::yaml_get_value<bool>(doc, "use_astar");
  allow_unknown_ = nav2_map_server::yaml_get_value<bool>(doc, "allow_unknown");
  inflated_distance_ = nav2_map_server::yaml_get_value<double>(doc, "inflated_distance");
  // use_final_approach_orientation_ = nav2_map_server::yaml_get_value<bool>(doc, "use_final_approach_orientation");

  // Create a planner based on the new costmap size
  planner_ = std::make_unique<NavFn>(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  // Initialize intention domain constraint (default: cone shape)
  constraint_ = intention_constraint::ConstraintFactory::create("cone");
}

std::vector<Point> NavfnPlannerWithCone::createPlan(
  const Point & start,
  const Point & goal)
{

  // we won't update the costmap when Agent training
  // Update planner based on the new costmap size
  // if (isPlannerOutOfDate()) {
  //   planner_->setNavArr(
  //     costmap_->getSizeInCellsX(),
  //     costmap_->getSizeInCellsY());
  // }

  std::vector<Point> path;

  // Corner case of the start(x,y) = goal(x,y)
  if (start.x == goal.x &&
    start.y == goal.y)
  {
    unsigned int mx, my;
    costmap_->worldToMap(start.x, start.y, mx, my);
    if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      // std::cout << "[Path Planner] Failed to create a unique pose path because of obstacles" << std::endl;
      return path;
    }
    Point pose;

    pose = start;

    return path;
  }

  if (!makePlan(start, goal, tolerance_, path)) {
    // std::cout << "[Path Planner] Failed to create a plan with tolerance " << tolerance_ << std::endl;
  }

  return path;
}

// bool
// NavfnPlannerWithCone::isPlannerOutOfDate()
// {
//   if (!planner_.get() ||
//     planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
//     planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()))
//   {
//     return true;
//   }
//   return false;
// }

bool
NavfnPlannerWithCone::makePlan(
  const Point & start,
  const Point & goal, double tolerance,
  std::vector<Point> & plan)
{
  // clear the plan, just in case
  plan.clear();

  double wx = start.x;
  double wy = start.y;

  // copy the costmap to costmap_for_plan_
  // TODO: update the old simulation env
  costmap_for_plan_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY(),
      costmap_->getResolution(),
      costmap_->getOriginX(),
      costmap_->getOriginY()
    );

  if (!costmap_for_plan_->copyWindow(*costmap_, 0, 0, costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), 0, 0)){
    // std::cout << "[Path Planner] Fail to copy costmap!" << std::endl;
  }

  try {
    if (enabled_){
      enabled_ = false;
      loadDomainToMap();
      // Clear cached parameters after use
      cur_pos_.clear();
      robot_direction_.clear();
      domain_params_.clear();
      }

      // make sure to resize the underlying array that Navfn uses
      planner_->setNavArr(
        costmap_for_plan_->getSizeInCellsX(),
        costmap_for_plan_->getSizeInCellsY());

      planner_->setCostmap(costmap_for_plan_->getCharMap(), true, allow_unknown_);

      // RCLCPP_INFO(logger_, "costmap_for_plan_ address is %p", static_cast<void*>(costmap_for_plan_.get()));
      // RCLCPP_INFO(logger_, "costmap_ address is %p", (void *)costmap_);
      // RCLCPP_INFO(logger_, "[After copying] current map size_x: %d, size_y %d", costmap_for_plan_->getSizeInCellsX(), costmap_for_plan_->getSizeInCellsY());
  } catch (const std::exception & e) {
    // std::cout << "[Path Planner] Caught exception: " << e.what() << std::endl;
    throw;
  }

  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    // std::cout << "[Path Planner] Cannot create a plan: the robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?" << std::endl;
    return false;
  }

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.x;
  wy = goal.y;

  if (!worldToMap(wx, wy, mx, my)) {
    // std::cout << "[Path Planner] Cannot create a plan: the goal is off the global costmap. Planning will always fail." << std::endl;
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  if (use_astar_) {
    planner_->calcNavFnAstar();
  } else {
    planner_->calcNavFnDijkstra(true);
  }

  double resolution = costmap_->getResolution();
  Point p, best_pose;

  bool found_legal = false;

  p = goal;
  double potential = getPointPotential(p);

  if (potential < POT_HIGH) {
    // Goal is reachable by itself
    best_pose = p;
    found_legal = true;
  } else {
    // Goal is not reachable. Trying to find nearest to the goal
    // reachable point within its tolerance region
    double best_sdist = std::numeric_limits<double>::max();

    p.y = goal.y - tolerance;
    while (p.y <= goal.y + tolerance) {
      p.x = goal.x - tolerance;
      while (p.x <= goal.x + tolerance) {
        potential = getPointPotential(p);
        double sdist = squared_distance(p, goal);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.x += resolution;
      }
      p.y += resolution;
    }
  }

  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      smoothApproachToGoal(best_pose, plan);

      // ============= WE don't consider the orientation of the robot here =============
      // If use_final_approach_orientation=true, interpolate the last pose orientation from the
      // previous pose to set the orientation to the 'final approach' orientation of the robot so
      // it does not rotate.
      // And deal with corner case of plan of length 1
      // if (use_final_approach_orientation_) {
      //   size_t plan_size = plan.size();
      //   if (plan_size == 1) {
      //     plan.back().orientation = start.orientation;
      //   } else if (plan_size > 1) {
      //     double dx, dy, theta;
      //     auto last_pose = plan.back();
      //     auto approach_pose = plan[plan_size - 2];
      //     // Deal with the case of NavFn producing a path with two equal last poses
      //     if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
      //       std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2)
      //     {
      //       approach_pose = plan[plan_size - 3];
      //     }
      //     dx = last_pose.x - approach_pose.x;
      //     dy = last_pose.y - approach_pose.y;
      //     theta = atan2(dy, dx);
      //     plan.back().orientation =
      //       nav2_util::geometry_utils::orientationAroundZAxis(theta);
      //   }
      // }
      // ==============================================================================
    } else {
      // std::cout << "[Path Planner] Failed to create a plan from potential when a legal potential was found. This shouldn't happen." << std::endl;
    }
  } else {
    // std::cout << "[Path Planner] Failed to find a legal plan from potential." << std::endl;
  }

  return !plan.empty();
}

void
NavfnPlannerWithCone::smoothApproachToGoal(
  const Point & goal,
  std::vector<Point> & plan)
{
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.
  if (plan.size() >= 2) {
    auto second_to_last_pose = plan.end()[-2];
    auto last_pose = plan.back();
    if (
      squared_distance(last_pose, second_to_last_pose) >
      squared_distance(goal, second_to_last_pose))
    {
      plan.back() = goal;
      return;
    }
  }
  Point goal_copy;
  goal_copy = goal;
  plan.push_back(goal_copy);
}

bool
NavfnPlannerWithCone::getPlanFromPotential(
  const Point & goal,
  std::vector<Point> & plan)
{
  // clear the plan, just in case
  plan.clear();

  // Goal should be in global frame
  double wx = goal.x;
  double wy = goal.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    // std::cout << "[Path Planner] The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal." << std::endl;
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = planner_->getLastPathCost();

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    Point pose;
    pose.x = world_x;
    pose.y = world_y;
    plan.push_back(pose);
  }

  // // debug
  // std::cout << "[Path Planner] The length of the plan is: " << plan.size() << std::endl;

  return !plan.empty();
}

double
NavfnPlannerWithCone::getPointPotential(const Point & world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}


bool
NavfnPlannerWithCone::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  // std::cout << "[Path Planner] worldToMap failed: mx,my: " << mx << "," << my << ", size_x,size_y: " << costmap_->getSizeInCellsX() << "," << costmap_->getSizeInCellsY() << std::endl;

  return false;
}

void
NavfnPlannerWithCone::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void
NavfnPlannerWithCone::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_for_plan_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

void
NavfnPlannerWithCone::loadIntentionDomain(
  std::vector<float> cur_pos,
  std::vector<float> robot_direction,
  std::vector<float> domain_params,
  bool is_enabled)
{
  enabled_ = is_enabled;
  if (!enabled_) {
    return;
  }

  // Validate input dimensions
  if (cur_pos.size() != 2) {
    std::cout << "[Load Intention Domain] cur_pos must have size 2" << std::endl;
    enabled_ = false;
    return;
  }

  if (robot_direction.size() != 2) {
    std::cout << "[Load Intention Domain] robot_direction must have size 2" << std::endl;
    enabled_ = false;
    return;
  }

  // Cache parameters for rendering in makePlan()
  cur_pos_ = cur_pos;
  robot_direction_ = robot_direction;
  domain_params_ = domain_params;

  return;
}

void
NavfnPlannerWithCone::loadDomainToMap()
{
  if (!constraint_) {
    std::cout << "[Load Domain To Map] Constraint not initialized" << std::endl;
    return;
  }

  // Step 1: Update constraint with cached parameters
  constraint_->updateParameters(cur_pos_, robot_direction_, domain_params_, inflated_distance_);

  // Step 2: Get bounding box in world coordinates
  float min_x, max_x, min_y, max_y;
  constraint_->getBoundingBox(min_x, max_x, min_y, max_y);

  // Step 3: Convert world bounding box to grid indices
  unsigned int min_mx, min_my, max_mx, max_my;
  bool min_valid = costmap_->worldToMap(min_x, min_y, min_mx, min_my);
  bool max_valid = costmap_->worldToMap(max_x, max_y, max_mx, max_my);

  // Clamp to valid map range
  min_mx = std::max(0u, std::min(min_mx, costmap_->getSizeInCellsX() - 1));
  max_mx = std::max(0u, std::min(max_mx, costmap_->getSizeInCellsX() - 1));
  min_my = std::max(0u, std::min(min_my, costmap_->getSizeInCellsY() - 1));
  max_my = std::max(0u, std::min(max_my, costmap_->getSizeInCellsY() - 1));

  // Step 4: Bounding box scan - iterate over all cells in the box
  for (unsigned int mx = min_mx; mx <= max_mx; ++mx) {
    for (unsigned int my = min_my; my <= max_my; ++my) {
      // Step 5: Convert grid cell center to world coordinates
      double wx, wy;
      costmap_->mapToWorld(mx, my, wx, wy);

      // Step 6: Query spatial predicate
      if (!constraint_->isRestrictedArea(static_cast<float>(wx), static_cast<float>(wy))) {
        // Mark as LETHAL_OBSTACLE (254)
        costmap_for_plan_->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
} 

float NavfnPlannerWithCone::getInflatedDistance()
{
  return inflated_distance_;
}

}  // namespace nav2_navfn_planner_with_cone
