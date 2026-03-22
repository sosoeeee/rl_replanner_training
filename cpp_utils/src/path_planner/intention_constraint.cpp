#include "path_planner/intention_constraint.hpp"

#include <algorithm>
#include <stdexcept>
#include <cmath>

namespace intention_constraint
{

//==============================================================================
// ConeIntentionConstraint Implementation
//==============================================================================

void ConeIntentionConstraint::updateParameters(
  const std::vector<float> & cur_pos,
  const std::vector<float> & robot_direction,
  const std::vector<float> & params,
  float inflated_distance)
{
  if (cur_pos.size() != 2 || robot_direction.size() != 2) {
    throw std::invalid_argument(
      "[ConeIntentionConstraint] cur_pos and robot_direction must have size 2");
  }

  if (params.size() < 2) {
    throw std::invalid_argument(
      "[ConeIntentionConstraint] params must have at least 2 elements [depth, radius]");
  }

  float depth = params[0];
  float radius = params[1];

  // Cone center (before inflation)
  std::vector<float> cone_center = {
    cur_pos[0] + depth * robot_direction[0],
    cur_pos[1] + depth * robot_direction[1]
  };

  // Inflate cone center forward
  std::vector<float> inflated_center = {
    cone_center[0] + robot_direction[0] * inflated_distance,
    cone_center[1] + robot_direction[1] * inflated_distance
  };

  // Inflate radius outward
  float phi = (radius > 0) ? std::atan(depth / radius) : M_PI / 2.0;
  float inflated_radius = (phi > 0) ?
    radius + inflated_distance / std::tan(phi / 2.0) :
    radius + inflated_distance;

  // Inflated base vertices (two points at cone base)
  Vertex base_v0, base_v1;
  base_v0.x = inflated_center[0] + inflated_radius * robot_direction[1];
  base_v0.y = inflated_center[1] - inflated_radius * robot_direction[0];
  base_v1.x = inflated_center[0] - inflated_radius * robot_direction[1];
  base_v1.y = inflated_center[1] + inflated_radius * robot_direction[0];

  // Inflated apex vertices (two points near robot position, pushed backward)
  float inflated_robot_x = cur_pos[0] - robot_direction[0] * inflated_distance;
  float inflated_robot_y = cur_pos[1] - robot_direction[1] * inflated_distance;
  float apex_offset = inflated_distance * std::tan(phi / 2.0);

  Vertex apex_v0, apex_v1;
  apex_v0.x = inflated_robot_x + apex_offset * robot_direction[1];
  apex_v0.y = inflated_robot_y - apex_offset * robot_direction[0];
  apex_v1.x = inflated_robot_x - apex_offset * robot_direction[1];
  apex_v1.y = inflated_robot_y + apex_offset * robot_direction[0];

  // Store vertices in order: apex_v0, base_v0, base_v1, apex_v1 (CCW traversal)
  polygon_vertices_.clear();
  polygon_vertices_.push_back(apex_v0);
  polygon_vertices_.push_back(base_v0);
  polygon_vertices_.push_back(base_v1);
  polygon_vertices_.push_back(apex_v1);

  parameters_initialized_ = true;
}

void ConeIntentionConstraint::getBoundingBox(
  float & min_x, float & max_x,
  float & min_y, float & max_y) const
{
  if (!parameters_initialized_) {
    throw std::runtime_error(
      "[ConeIntentionConstraint] Cannot get bounding box: parameters not initialized");
  }

  // Compute axis-aligned bounding box from all trapezoid vertices
  min_x = polygon_vertices_[0].x;
  max_x = polygon_vertices_[0].x;
  min_y = polygon_vertices_[0].y;
  max_y = polygon_vertices_[0].y;

  for (size_t i = 1; i < polygon_vertices_.size(); ++i) {
    min_x = std::min(min_x, polygon_vertices_[i].x);
    max_x = std::max(max_x, polygon_vertices_[i].x);
    min_y = std::min(min_y, polygon_vertices_[i].y);
    max_y = std::max(max_y, polygon_vertices_[i].y);
  }
}

bool ConeIntentionConstraint::isRestrictedArea(float x, float y) const
{
  if (!parameters_initialized_) {
    return false;  // If not initialized, no restricted area
  }

  // Point-in-polygon test using cross products (same as Python implementation)
  // Trapezoid is convex with vertices in CCW order: apex_v0, base_v0, base_v1, apex_v1
  //
  // Algorithm: For each edge, compute cross product of edge vector and point vector.
  // If all cross products >= 0 (all on left/inside), point is inside polygon.

  for (size_t i = 0; i < 4; ++i) {
    const Vertex & v_curr = polygon_vertices_[i];
    const Vertex & v_next = polygon_vertices_[(i + 1) % 4];

    // Edge vector: v_curr -> v_next
    float edge_x = v_next.x - v_curr.x;
    float edge_y = v_next.y - v_curr.y;

    // Vector from current vertex to test point: v_curr -> (x, y)
    float to_point_x = x - v_curr.x;
    float to_point_y = y - v_curr.y;

    // 2D cross product: edge × to_point = edge_x * to_point_y - edge_y * to_point_x
    float cross = edge_x * to_point_y - edge_y * to_point_x;

    // For CCW polygon, all cross products should be >= 0 for interior points
    if (cross < 0.0f) {
      return false;  // Point is outside
    }
  }

  return true;  // Point is inside or on boundary
}

//==============================================================================
// RectangleIntentionConstraint Implementation
//==============================================================================

void RectangleIntentionConstraint::updateParameters(
  const std::vector<float> & cur_pos,
  const std::vector<float> & robot_direction,
  const std::vector<float> & params,
  float inflated_distance)
{
  if (cur_pos.size() != 2 || robot_direction.size() != 2) {
    throw std::invalid_argument(
      "[RectangleIntentionConstraint] cur_pos and robot_direction must have size 2");
  }

  if (params.size() < 2) {
    throw std::invalid_argument(
      "[RectangleIntentionConstraint] params must have at least 2 elements [depth, radius]");
  }

  float depth = params[0];
  float radius = params[1];

  // Rectangle center (before inflation)
  std::vector<float> rectangle_center = {
    cur_pos[0] + depth * robot_direction[0],
    cur_pos[1] + depth * robot_direction[1]
  };

  // Inflate rectangle center forward
  std::vector<float> inflated_center = {
    rectangle_center[0] + robot_direction[0] * inflated_distance,
    rectangle_center[1] + robot_direction[1] * inflated_distance
  };

  // Inflate radius outward
  float inflated_radius = radius + inflated_distance;

  // Inflated base vertices (two points at rectangle base)
  Vertex base_v0, base_v1;
  base_v0.x = inflated_center[0] + inflated_radius * robot_direction[1];
  base_v0.y = inflated_center[1] - inflated_radius * robot_direction[0];
  base_v1.x = inflated_center[0] - inflated_radius * robot_direction[1];
  base_v1.y = inflated_center[1] + inflated_radius * robot_direction[0];

  // Inflated apex vertices (two points near robot position, pushed backward)
  float inflated_robot_x = cur_pos[0] - robot_direction[0] * inflated_distance;
  float inflated_robot_y = cur_pos[1] - robot_direction[1] * inflated_distance;

  Vertex apex_v0, apex_v1;
  apex_v0.x = inflated_robot_x + inflated_radius * robot_direction[1];
  apex_v0.y = inflated_robot_y - inflated_radius * robot_direction[0];
  apex_v1.x = inflated_robot_x - inflated_radius * robot_direction[1];
  apex_v1.y = inflated_robot_y + inflated_radius * robot_direction[0];

  // Store vertices in order: apex_v0, base_v0, base_v1, apex_v1 (CCW traversal)
  polygon_vertices_.clear();
  polygon_vertices_.push_back(apex_v0);
  polygon_vertices_.push_back(base_v0);
  polygon_vertices_.push_back(base_v1);
  polygon_vertices_.push_back(apex_v1);

  parameters_initialized_ = true;
}

//==============================================================================
// ConstraintFactory Implementation
//==============================================================================

std::unique_ptr<BaseIntentionConstraint> ConstraintFactory::create(
  const std::string & constraint_type)
{
  if (constraint_type == "cone") {
    return std::make_unique<ConeIntentionConstraint>();
  }
  // Future shapes can be added here:
  else if (constraint_type == "rectangle") {
    return std::make_unique<RectangleIntentionConstraint>();
  }
  // else if (constraint_type == "ellipse") {
  //   return std::make_unique<EllipseIntentionConstraint>();
  // }
  else {
    throw std::invalid_argument(
      "[ConstraintFactory] Unknown constraint type: " + constraint_type +
      ". Available types: cone");
  }
}

}  // namespace intention_constraint
