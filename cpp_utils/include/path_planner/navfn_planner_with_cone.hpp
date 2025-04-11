#ifndef NAV2_NAVFN_PLANNER__NAVFN_PLANNER_WITH_CONE_HPP_
#define NAV2_NAVFN_PLANNER__NAVFN_PLANNER_WITH_CONE_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "path_planner/navfn.hpp"
#include "map_loader/costmap_2d.hpp"
#include "utils.h"

// // Publish the costmap loaded cone to debug
// #define DEBUG

namespace nav2_navfn_planner_with_cone
{

class NavfnPlannerWithCone 
{
public:
  /**
   * @brief constructor
   */
  NavfnPlannerWithCone();

  /**
   * @brief destructor
   */
  ~NavfnPlannerWithCone();

  /**
   * @brief Configuring path planner based on yaml file
   * @param yaml_filename Path to the yaml file
   */
  void configure(nav2_costmap_2d::Costmap2D * costmap, const std::string & yaml_filename);

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav_msgs::Path of the generated path
   */
  std::vector<Point> createPlan(
    const Point & start,
    const Point & goal);

  void loadCone(std::vector<float> center, std::vector<float> current_pos, float radius, bool is_enabled = true);

  float getInflatedDistance();

protected:
  /**
   * @brief Compute a plan given start and goal poses, provided in global world frame.
   * @param start Start pose
   * @param goal Goal pose
   * @param tolerance Relaxation constraint in x and y
   * @param plan Path to be computed
   * @return true if can find the path
   */
  bool makePlan(
    const Point & start,
    const Point & goal, double tolerance,
    std::vector<Point> & plan);

  /**
   * @brief Compute a plan to a goal from a potential - must call computePotential first
   * @param goal Goal pose
   * @param plan Path to be computed
   * @return true if can compute a plan path
   */
  bool getPlanFromPotential(
    const Point & goal,
    std::vector<Point> & plan);

  /**
   * @brief Remove artifacts at the end of the path - originated from planning on a discretized world
   * @param goal Goal pose
   * @param plan Computed path
   */
  void smoothApproachToGoal(
    const Point & goal,
    std::vector<Point> & plan);

  /**
   * @brief Compute the potential, or navigation cost, at a given point in the world
   *        must call computePotential first
   * @param world_point Point in world coordinate frame
   * @return double point potential (navigation cost)
   */
  double getPointPotential(const Point & world_point);

  // Check for a valid potential value at a given point in the world
  // - must call computePotential first
  // - currently unused
  // bool validPointPotential(const geometry_msgs::msg::Point & world_point);
  // bool validPointPotential(const geometry_msgs::msg::Point & world_point, double tolerance);

  /**
   * @brief Compute the squared distance between two points
   * @param p1 Point 1
   * @param p2 Point 2
   * @return double squared distance between two points
   */
  inline double squared_distance(
    const Point & p1,
    const Point & p2)
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return dx * dx + dy * dy;
  }

  /**
   * @brief Transform a point from world to map frame
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   * @param mx int of map X coordinate
   * @param my int of map Y coordinate
   * @return true if can transform
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  /**
   * @brief Transform a point from map to world frame
   * @param mx double of map X coordinate
   * @param my double of map Y coordinate
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   */
  void mapToWorld(double mx, double my, double & wx, double & wy);

  /**
   * @brief Set the corresponding cell cost to be free space
   * @param mx int of map X coordinate
   * @param my int of map Y coordinate
   */
  void clearRobotCell(unsigned int mx, unsigned int my);

  // Planner based on ROS1 NavFn algorithm
  std::unique_ptr<NavFn> planner_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // Whether or not the planner should be allowed to plan through unknown space
  bool allow_unknown_;
  // bool use_final_approach_orientation_;

  // If the goal is obstructed, the tolerance specifies how many meters the planner
  // can relax the constraint in x and y before failing
  double tolerance_;

  // Whether to use the astar planner or default dijkstras
  bool use_astar_;

  // ======================================================================================== //
  // Cone related parameters
  bool enabled_;
  std::vector<float> center_; // Center of the cone
  std::vector<float> cur_pos_; // cur_pos of the robot (get from predictor server)
  float radius_; // Radius of the cone
  float inflated_distance_;
  double resolution_{0};

  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_for_plan_; // a copy

  void loadConeToMap(float robot_x, float robot_y);
  void setEdgeCost(float wx0, float wy0, float wx1, float wy1, unsigned char cost_value);
};

}  // namespace nav2_navfn_planner_with_cone

#endif  // NAV2_NAVFN_PLANNER__NAVFN_PLANNER_WITH_CONE_HPP_
