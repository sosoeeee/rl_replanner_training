#ifndef INTENTION_CONSTRAINT_HPP_
#define INTENTION_CONSTRAINT_HPP_

#include <vector>
#include <memory>
#include <string>
#include <cmath>

namespace intention_constraint
{

/**
 * @brief Abstract base class for intention domain geometric constraints.
 *
 * Defines interfaces for:
 * 1. Parameter updates (position, direction, shape params, inflation)
 * 2. Bounding box queries (for efficient grid scanning)
 * 3. Spatial predicates (for point-in-domain tests)
 *
 * This abstraction enables Strategy Pattern for different domain shapes
 * (cone, rectangle, ellipse, etc.) without modifying planner core logic.
 */
class BaseIntentionConstraint
{
public:
  BaseIntentionConstraint() = default;
  virtual ~BaseIntentionConstraint() = default;

  std::vector<float> transformWorldToLocal(const std::vector<float> & world_point) const
  {
    // Transform point to ellipse-centered frame
    float dx = world_point[0] - _cur_pos[0];
    float dy = world_point[1] - _cur_pos[1];

    float local_x = dx * _robot_direction[0] + dy * _robot_direction[1];
    float local_y = -dx * _robot_direction[1] + dy * _robot_direction[0];

    return {local_x, local_y};
  }

  std::vector<float> transformLocalToWorld(const std::vector<float> & local_point) const
  {
    // Transform point from ellipse-centered frame to world frame
    float world_x = _cur_pos[0] + local_point[0] * _robot_direction[0] - local_point[1] * _robot_direction[1];
    float world_y = _cur_pos[1] + local_point[0] * _robot_direction[1] + local_point[1] * _robot_direction[0];

    return {world_x, world_y};
  }

  /**
   * @brief Update constraint parameters.
   *
   * Separates three independent parameter categories:
   * - cur_pos, robot_direction: spatial transform (origin and orientation)
   * - params: shape-specific geometry (e.g., [depth, radius] for cone)
   * - inflated_distance: safety margin expansion
   * - resolution: map resolution for trajectory discretization
   *
   * @param cur_pos Current robot position [x, y]
   * @param robot_direction Normalized direction vector [dx, dy]
   * @param params Shape-specific parameters (generic float vector)
   * @param inflated_distance Inflation margin for robot safety
   * @param resolution Map resolution for trajectory discretization (used by corridor)
   */
  virtual void updateParameters(
    const std::vector<float> & cur_pos,
    const std::vector<float> & robot_direction,
    const std::vector<float> & params,
    float inflated_distance,
    float resolution = 0.0f) = 0;

  /**
   * @brief Get axis-aligned bounding box in world coordinates.
   *
   * Used to limit grid scan range in costmap rendering.
   *
   * @param min_x Minimum x coordinate (output)
   * @param max_x Maximum x coordinate (output)
   * @param min_y Minimum y coordinate (output)
   * @param max_y Maximum y coordinate (output)
   */
  virtual void getBoundingBox(
    float & min_x, float & max_x,
    float & min_y, float & max_y) const = 0;

  /**
   * @brief Spatial predicate: check if point is in restricted area.
   *
   * CRITICAL: This method must have identical logic to Python's
   * `is_restricted_area()` to ensure rendering consistency.
   *
   * @param x World x coordinate
   * @param y World y coordinate
   * @return True if point is on or inside the inflated domain boundary
   */
  virtual bool isRestrictedArea(float x, float y) const = 0;

protected:
  // frame transformation utilities can be added here if needed
  std::vector<float> _cur_pos;
  std::vector<float> _robot_direction;
};

/**
 * @brief Cone intention constraint implementation.
 *
 * Parameters:
 *   params[0] = depth: distance from robot to cone base center
 *   params[1] = radius: half-width of cone base
 *
 * Geometry:
 *   - Apex at cur_pos (pushed back by inflated_distance)
 *   - Base at cur_pos + depth * robot_direction (pushed forward by inflated_distance)
 *   - Forms a trapezoid after inflation
 *
 * Mathematical consistency requirement:
 *   - Must match Python's ConeIntentionDomain.is_restricted_area() exactly
 *   - Uses cross product half-plane tests for convex polygon containment
 */
class ConeIntentionConstraint : public BaseIntentionConstraint
{
public:
  ConeIntentionConstraint() = default;
  ~ConeIntentionConstraint() override = default;

  void updateParameters(
    const std::vector<float> & cur_pos,
    const std::vector<float> & robot_direction,
    const std::vector<float> & params,
    float inflated_distance,
    float resolution = 0.0f) override;

  void getBoundingBox(
    float & min_x, float & max_x,
    float & min_y, float & max_y) const override;

  bool isRestrictedArea(float x, float y) const override;

protected:
  // Cached inflated polygon vertices
  struct Vertex {
    float x;
    float y;
  };

  std::vector<Vertex> polygon_vertices_;  // [apex_v0, base_v0, base_v1, apex_v1]
  bool parameters_initialized_ = false;
};

/**
 * @brief Rectangle intention constraint implementation.
 * 
 * Parameters:
 *   params[0] = depth: distance from robot to rectangle center
 *   params[1] = radius: half-width of rectangle (perpendicular to robot direction)
 *
 */
class RectangleIntentionConstraint : public ConeIntentionConstraint
{
public:
  RectangleIntentionConstraint() = default;
  ~RectangleIntentionConstraint() override = default;

  void updateParameters(
    const std::vector<float> & cur_pos,
    const std::vector<float> & robot_direction,
    const std::vector<float> & params,
    float inflated_distance,
    float resolution = 0.0f) override;
};

/**
 * @brief Ellipse intention constraint implementation.
 * 
 * Parameters:
 *   params[0] = depth: distance from robot to ellipse center
 *   params[1] = radius: half-width of ellipse (perpendicular to robot direction)
 *
 */
class EllipseIntentionConstraint : public RectangleIntentionConstraint
{
public:
  EllipseIntentionConstraint() = default;
  ~EllipseIntentionConstraint() override = default;
  
  void updateParameters(
    const std::vector<float> & cur_pos,
    const std::vector<float> & robot_direction,
    const std::vector<float> & params,
    float inflated_distance,
    float resolution = 0.0f) override;

  bool isRestrictedArea(float x, float y) const override;

private:
  // Additional members for ellipse-specific geometry can be added here
  float _inflated_a;  // Semi-major axis after inflation
  float _inflated_b;  // Semi-minor axis after inflation
  float _a; 
};

/**
 * @brief Corridor intention constraint implementation.
 *
 * Parameters:
 *   params[0] = S: trajectory length coefficient
 *   params[1] = r: corridor radius (half-width)
 *   params[2] = w0: initial angular offset
 *   params[3] = v0: velocity parameter (passed through action params)
 *
 * The corridor is composed of multiple circles along a sigmoid trajectory.
 */
class CorridorIntentionConstraint : public BaseIntentionConstraint
{
public:
  explicit CorridorIntentionConstraint(float lambda = 0.95f)
    : _lambda(lambda), _trajectory_length(0.0f) {}
  ~CorridorIntentionConstraint() override = default;

  void updateParameters(
    const std::vector<float> & cur_pos,
    const std::vector<float> & robot_direction,
    const std::vector<float> & params,
    float inflated_distance,
    float resolution = 0.0f) override;

  void getBoundingBox(
    float & min_x, float & max_x,
    float & min_y, float & max_y) const override;

  bool isRestrictedArea(float x, float y) const override;

private:
  /**
   * @brief Calculate trajectory centerline using sigmoid function.
   *
   * Trajectory angle: theta(s) = (w0/_lambda) * (1 - exp(-_lambda * s))
   *
   * @param params [S, r, w0, v0]
   * @param resolution Map resolution for trajectory discretization
   */
  void calculateTrajectory(const std::vector<float> & params, float resolution);

  /**
   * @brief Build corridor circles along trajectory for collision checking.
   *
   * @param inflated_radius Inflated corridor radius
   * @param resolution Map resolution for corridor spacing
   */
  void buildCorridors(float inflated_radius, float resolution);

  float _lambda;                                        // Curvature control parameter
  float _trajectory_length;                             // Total trajectory length
  std::vector<std::vector<float>> _trajectory;          // Centerline trajectory points: [[x, y], ...]
  std::vector<std::vector<float>> _corridors;           // Corridor circles: [[x, y, r], ...]
  bool parameters_initialized_ = false;
};

/**
 * @brief Factory for creating intention constraint instances.
 *
 * Usage:
 *   auto constraint = ConstraintFactory::create("cone");
 *   constraint->updateParameters(cur_pos, direction, params, inflation);
 *   if (constraint->isRestrictedArea(wx, wy)) { ... }
 */
class ConstraintFactory
{
public:
  /**
   * @brief Create constraint by type name.
   *
   * @param constraint_type Type identifier ("cone", "rectangle", "ellipse", "corridor")
   * @return Unique pointer to constraint instance
   * @throws std::invalid_argument if type is unknown
   */
  static std::unique_ptr<BaseIntentionConstraint> create(
    const std::string & constraint_type);
};

}  // namespace intention_constraint

#endif  // INTENTION_CONSTRAINT_HPP_
