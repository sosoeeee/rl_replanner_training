"""
Intention Domain Module

Implements the Strategy Pattern for intention domains, allowing flexible
geometry shapes (cone, rectangle, ellipse, etc.) for human intention prediction.

Architecture:
- BaseIntentionDomain: Abstract base class defining the interface
- ConeIntentionDomain: Concrete implementation for closed cone shape
- IntentionDomainFactory: Factory for creating domain instances
"""

import abc
import math
import numpy as np
from typing import Dict, Optional, Tuple, List, Callable


class BaseIntentionDomain(abc.ABC):
    """
    Abstract base class for intention domain geometries.

    Defines interfaces for:
    1. Environment interaction (action space, goal prediction, regularization)
    2. Physical rendering alignment (bounding box, spatial predicates, visualization)
    """

    def __init__(self):
        """Initialize the intention domain with default private attributes."""
        self._action_params: Optional[List[float]] = None
        self._cur_pos: Optional[List[float]] = None
        self._robot_direction: Optional[np.ndarray] = None

    def configure(
        self,
        action_params: List[float],
        cur_pos: List[float],
        robot_direction: np.ndarray
    ) -> None:
        """
        Configure the intention domain with generation parameters.

        Args:
            action_params: Shape-specific parameters (e.g., [depth, radius] for cone)
            cur_pos: Current robot position [x, y]
            robot_direction: Normalized direction vector [dx, dy]
        """
        self._action_params = action_params
        self._cur_pos = cur_pos
        self._robot_direction = robot_direction

    @abc.abstractmethod
    def get_action_space_setting(self) -> Dict[str, List[float]]:
        """
        Returns the action space parameter ranges for this domain shape.

        Returns:
            Dict mapping parameter names to [min, max] ranges.
            Example: {'depth': [1e-3, 0.707], 'radius': [1e-3, 0.707]}
        """
        pass

    @abc.abstractmethod
    def get_predicted_goal(
        self,
        global_goal: Optional[List[float]] = None,
        collision_checker: Optional[Callable[[List[float]], bool]] = None,
        map_resolution: Optional[float] = None
    ) -> Optional[Tuple[List[float], Dict]]:
        """
        Compute the predicted subgoal within this intention domain.

        Args:
            global_goal: Global goal position [x, y]
            collision_checker: Function to check if a point is in collision
            map_resolution: Grid resolution for obstacle avoidance ray casting

        Returns:
            Tuple of (predicted_goal, auxiliary_data) if successful, None if failed.
            - predicted_goal: [x, y] coordinates of the predicted goal
            - auxiliary_data: Dict containing intermediate results for rendering
        """
        pass

    @abc.abstractmethod
    def get_reg_reward(self) -> float:
        """
        Calculate regularization reward based on domain parameters.

        Returns:
            Regularization reward value (typically negative to penalize large domains)
        """
        pass

    @abc.abstractmethod
    def get_bounding_box(
        self,
        inflated_distance: Optional[float] = None
    ) -> Tuple[float, float, float, float]:
        """
        Get the axis-aligned bounding box of the inflated domain in world coordinates.

        Args:
            inflated_distance: Inflation distance for robot safety margin

        Returns:
            (min_x, max_x, min_y, max_y) in world coordinates
        """
        pass

    @abc.abstractmethod
    def is_restricted_area(
        self,
        wx: float,
        wy: float,
    ) -> bool:
        """
        Spatial predicate: check if a world point is within the inflated domain boundary.

        This method is critical for ensuring Python and C++ rendering consistency.

        Args:
            wx, wy: World coordinates to test
            inflated_distance: Inflation distance for robot safety margin

        Returns:
            True if the point is on or inside the domain boundary
        """
        pass

    @abc.abstractmethod
    def get_visualization_polygon(
        self
    ) -> List[List[float]]:
        """
        Get polygon vertices for RViz line strip visualization.

        Returns:
            List of [x, y] vertices forming a closed polygon (first == last)
        """
        pass


class ConeIntentionDomain(BaseIntentionDomain):
    """
    Closed cone intention domain implementation.

    Parameters:
        - depth: Distance from robot to cone base center (forward projection)
        - radius: Half-width of the cone base

    Geometry:
        Robot position (cur_pos) -> depth along robot_direction -> cone center
        Cone base is perpendicular to robot_direction with width 2*radius
    """

    def __init__(self):
        """Initialize the cone intention domain."""
        super().__init__()
        self._inflated_robot_vertices: Optional[List[Dict[str, float]]] = None
        self._inflated_base_vertices: Optional[List[Dict[str, float]]] = None   

    def configure(self, action_params, cur_pos, robot_direction):
        """
        Configure the cone intention domain with parameters.

        Args:
            action_params: [depth, radius] for the cone
            cur_pos: Current robot position [x, y]
            robot_direction: Normalized direction vector [dx, dy]
        """
        super().configure(action_params, cur_pos, robot_direction)
        self._inflated_robot_vertices = None  # Reset inflated geometry cache
        self._inflated_base_vertices = None

    def get_action_space_setting(self) -> Dict[str, List[float]]:
        """
        Returns cone-specific action space parameters.

        Ranges normalized to [0, 1] during action sampling, then scaled by obser_width.
        """
        return {
            'depth': [1e-3, np.sqrt(2) / 2],    # Max depth ~0.707 * obser_width
            'radius': [1e-3, np.sqrt(2) / 2]    # Max radius ~0.707 * obser_width
        }

    def get_predicted_goal(
        self,
        global_goal: Optional[List[float]] = None,
        collision_checker: Optional[Callable[[List[float]], bool]] = None,
        map_resolution: Optional[float] = None
    ) -> Optional[Tuple[List[float], Dict]]:
        """
        Compute predicted goal within cone domain.

        Args:
            global_goal: Global goal position [x, y]
            collision_checker: Function to check if a point is in collision
            map_resolution: Grid resolution for obstacle avoidance ray casting

        Algorithm:
        1. Compute cone center and base vertices
        2. Project global goal onto cone base edge (perpendicular drop)
        3. Select closest point to global goal on base edge
        4. Avoid obstacles from cone center toward selected point

        Returns:
            Tuple of ([pred_x, pred_y], {'cone_center': [x, y]}) or None if failed
        """
        # Use configured values if parameters not provided
        action_params = self._action_params
        cur_pos = self._cur_pos
        robot_direction = self._robot_direction

        if action_params is None or cur_pos is None or robot_direction is None:
            raise ValueError("action_params, cur_pos, and robot_direction must be configured or provided")

        depth, radius = action_params[0], action_params[1]

        # Compute cone center
        cone_center = [
            cur_pos[0] + depth * robot_direction[0],
            cur_pos[1] + depth * robot_direction[1]
        ]

        # Compute base vertices (perpendicular to robot direction)
        vertices = []
        for i in range(2):
            x = cone_center[0] + radius * robot_direction[1] * math.cos(i * math.pi)
            y = cone_center[1] - radius * robot_direction[0] * math.cos(i * math.pi)
            vertices.append({'x': x, 'y': y})

        # Project global goal onto base edge (perpendicular intersection)
        global_x, global_y = global_goal[0], global_goal[1]
        v0, v1 = vertices[0], vertices[1]

        # Perpendicular intersection formula
        inter_x = ((global_y - v0['y']) * (v1['y'] - v0['y']) * (v1['x'] - v0['x']) +
                   global_x * (v1['x'] - v0['x']) * (v1['x'] - v0['x']) +
                   v0['x'] * (v1['y'] - v0['y']) * (v1['y'] - v0['y'])) / \
                  ((v1['y'] - v0['y']) * (v1['y'] - v0['y']) + (v1['x'] - v0['x']) * (v1['x'] - v0['x']))
        inter_y = (v0['x'] - v1['x']) / (v1['y'] - v0['y']) * (inter_x - global_x) + global_y

        # Determine which point on base edge is closest to global goal
        vector_0 = np.array([global_x - v0['x'], global_y - v0['y']])
        module_0 = vector_0.dot(vector_0) ** 0.5
        vector_1 = np.array([global_x - v1['x'], global_y - v1['y']])
        module_1 = vector_1.dot(vector_1) ** 0.5

        # Base edge direction (perpendicular to robot direction)
        base_direction = np.array([robot_direction[1], -robot_direction[0]])
        cos_0 = vector_0.dot(base_direction) / module_0 if module_0 > 0 else 0
        cos_1 = vector_1.dot(base_direction) / module_1 if module_1 > 0 else 0

        cone_center_dict = {'x': cone_center[0], 'y': cone_center[1]}

        try:
            if cos_0 * cos_1 > 0:
                # Unilateral case: global goal projects outside base edge
                if abs(cos_0) < abs(cos_1):
                    # Vertex 0 is closer to global goal
                    pred_position = self._avoid_obstacles_from_center(
                        cone_center_dict, v0, radius, collision_checker, map_resolution
                    )
                else:
                    # Vertex 1 is closer to global goal
                    pred_position = self._avoid_obstacles_from_center(
                        cone_center_dict, v1, radius, collision_checker, map_resolution
                    )
            else:
                # Bilateral case: global goal projects within base edge
                pred_position = self._avoid_obstacles_from_center(
                    cone_center_dict, {'x': inter_x, 'y': inter_y}, radius, collision_checker, map_resolution
                )
        except Exception as e:
            # Failed to compute predicted goal
            return None

        if pred_position is None:
            return None

        return (pred_position, None)

    def _avoid_obstacles_from_center(
        self,
        center: Dict[str, float],
        target: Dict[str, float],
        max_distance: float,
        collision_checker: Callable[[List[float]], bool],
        map_resolution: float
    ) -> Optional[List[float]]:
        """
        Find collision-free point closest to target on ray from center to target.

        Args:
            center: Ray origin {'x': float, 'y': float}
            target: Ray target {'x': float, 'y': float}
            max_distance: Maximum search distance (typically radius)
            collision_checker: Function to check collision at [x, y]
            map_resolution: Grid resolution for ray casting step size

        Returns:
            [x, y] of collision-free point, or None if entire ray is blocked
        """
        module = math.sqrt((target['x'] - center['x']) ** 2 + (target['y'] - center['y']) ** 2)
        if module < 1e-6:
            return None

        dir_x = (target['x'] - center['x']) / module
        dir_y = (target['y'] - center['y']) / module

        distance = 0
        p = [center['x'] + distance * dir_x, center['y'] + distance * dir_y]

        # If center is in collision, move outward to find free space
        while collision_checker(p):
            distance += map_resolution
            if distance > max_distance:
                # Try opposite direction as fallback
                dir_x = -dir_x
                dir_y = -dir_y
                distance = map_resolution
                p = [center['x'] + distance * dir_x, center['y'] + distance * dir_y]

                while distance < max_distance:
                    if not collision_checker(p):
                        return p
                    distance += map_resolution
                    p = [center['x'] + distance * dir_x, center['y'] + distance * dir_y]

                return None
            else:
                p = [center['x'] + distance * dir_x, center['y'] + distance * dir_y]

        # Now in free space, advance until collision or max_distance
        max_distance = min(max_distance, module)  # Don't go beyond target
        while distance < max_distance:
            distance += map_resolution
            p = [center['x'] + distance * dir_x, center['y'] + distance * dir_y]
            if not collision_checker(p):
                continue
            else:
                # Hit obstacle, return previous free point
                return [center['x'] + (distance - map_resolution) * dir_x,
                        center['y'] + (distance - map_resolution) * dir_y]

        # Reached max_distance without collision
        return [center['x'] + distance * dir_x,
                center['y'] + distance * dir_y]

    def get_reg_reward(self) -> float:
        """
        Cone regularization reward penalizes wide opening angle.

        Uses logarithmic penalty on normalized angle (atan(radius/depth)).
        """
        # Use configured values if parameters not provided
        action_params = self._action_params

        if action_params is None:
            raise ValueError("action_params must be configured or provided")

        depth, radius = action_params[0], action_params[1]

        # Normalized opening angle: arctan(radius/depth) / (pi/2)
        norm_angle = np.arctan(radius / depth) / (np.pi / 2)

        # Logarithmic penalty: log(1 - norm_angle) / (1 - norm_angle)
        # As norm_angle -> 1 (wide cone), penalty -> -inf
        reg_reward = np.log(1 - norm_angle) / (1 - norm_angle)

        return reg_reward

    def get_bounding_box(
        self,
        inflated_distance: Optional[float] = None
    ) -> Tuple[float, float, float, float]:
        """
        Compute axis-aligned bounding box of inflated cone.

        Inflated cone is a trapezoid with:
        - Top edge: inflated robot vertices (apex pushed back)
        - Bottom edge: inflated base vertices (base pushed forward)
        - Side edges: connecting top and bottom
        """
        # Use configured values if parameters not provided
        action_params = self._action_params
        cur_pos = self._cur_pos
        robot_direction = self._robot_direction

        if action_params is None or cur_pos is None or robot_direction is None or inflated_distance is None:
            raise ValueError("action_params, cur_pos, robot_direction, and inflated_distance must be provided")

        depth, radius = action_params[0], action_params[1]

        # Cone center (before inflation)
        cone_center = [
            cur_pos[0] + depth * robot_direction[0],
            cur_pos[1] + depth * robot_direction[1]
        ]

        # Inflate cone center forward
        inflated_center = [
            cone_center[0] + robot_direction[0] * inflated_distance,
            cone_center[1] + robot_direction[1] * inflated_distance
        ]

        # Inflate radius outward
        phi = math.atan(depth / radius) if radius > 0 else math.pi / 2
        inflated_radius = radius + inflated_distance / math.tan(phi / 2) if phi > 0 else radius + inflated_distance

        # Inflated base vertices
        self._inflated_base_vertices = []
        for i in range(2):
            x = inflated_center[0] + inflated_radius * robot_direction[1] * math.cos(i * math.pi)
            y = inflated_center[1] - inflated_radius * robot_direction[0] * math.cos(i * math.pi)
            self._inflated_base_vertices.append({'x': x, 'y': y})

        # Inflated apex vertices (pushed backward)
        inflated_robot_x = cur_pos[0] - robot_direction[0] * inflated_distance
        inflated_robot_y = cur_pos[1] - robot_direction[1] * inflated_distance
        self._inflated_robot_vertices = []
        for i in range(2):
            x = inflated_robot_x + inflated_distance * math.tan(phi / 2) * robot_direction[1] * math.cos(i * math.pi)
            y = inflated_robot_y - inflated_distance * math.tan(phi / 2) * robot_direction[0] * math.cos(i * math.pi)
            self._inflated_robot_vertices.append({'x': x, 'y': y})

        # Compute axis-aligned bounding box from all 4 vertices
        all_x = [v['x'] for v in self._inflated_base_vertices] + [v['x'] for v in self._inflated_robot_vertices]
        all_y = [v['y'] for v in self._inflated_base_vertices] + [v['y'] for v in self._inflated_robot_vertices]

        return (min(all_x), max(all_x), min(all_y), max(all_y))

    def is_restricted_area(
        self,
        wx: float,
        wy: float,
    ) -> bool:
        """
        Check if point (wx, wy) is inside or on the inflated cone boundary.

        Uses local coordinate transformation and cross product for half-plane tests.

        Algorithm:
        1. Transform to local frame with origin at cur_pos, x-axis along robot_direction
        2. Check if point is within trapezoid formed by inflated cone edges
        3. Use cross product to test half-plane containment
        """
        # Use configured values if parameters not provided
        if self._inflated_base_vertices is None or self._inflated_robot_vertices is None:
            raise ValueError("Inflated geometry must be computed by calling get_bounding_box with inflated_distance before using is_restricted_area")

        # Check if point is inside trapezoid using cross products
        # Trapezoid vertices in order: apex_v0, base_v0, base_v1, apex_v1
        vertices_ordered = [
            self._inflated_robot_vertices[0],
            self._inflated_base_vertices[0],
            self._inflated_base_vertices[1],
            self._inflated_robot_vertices[1]
        ]

        # Point-in-polygon test using cross products
        for i in range(4):
            v_curr = vertices_ordered[i]
            v_next = vertices_ordered[(i + 1) % 4]

            # Edge vector
            edge_x = v_next['x'] - v_curr['x']
            edge_y = v_next['y'] - v_curr['y']

            # Vector from current vertex to test point
            to_point_x = wx - v_curr['x']
            to_point_y = wy - v_curr['y']

            # Cross product (2D): edge × to_point
            cross = edge_x * to_point_y - edge_y * to_point_x

            # For convex polygon traversed CCW, all cross products should be >= 0
            # If any cross product < 0, point is outside
            if cross < 0:
                return False

        return True

    def get_visualization_polygon(
        self,
    ) -> List[List[float]]:
        """
        Get inflated cone vertices for LINE_STRIP visualization.

        Returns vertices in order: apex_v0 -> base_v0 -> base_v1 -> apex_v1 -> apex_v0 (closed loop)
        """
        # Use configured values if parameters not provided
        action_params = self._action_params
        cur_pos = self._cur_pos
        robot_direction = self._robot_direction

        if action_params is None or cur_pos is None or robot_direction is None:
            raise ValueError("action_params, cur_pos, and robot_direction must be configured or provided")

        depth, radius = action_params[0], action_params[1]

        # The visualization does NOT use inflated geometry, use original cone
        # (Inflation is only for costmap marking, not visual marker)
        cone_center = [
            cur_pos[0] + depth * robot_direction[0],
            cur_pos[1] + depth * robot_direction[1]
        ]

        # Original base vertices
        base_vertices = []
        for i in range(2):
            x = cone_center[0] + radius * robot_direction[1] * math.cos(i * math.pi)
            y = cone_center[1] - radius * robot_direction[0] * math.cos(i * math.pi)
            base_vertices.append([x, y])

        # Apex at robot position
        apex = [cur_pos[0], cur_pos[1]]

        # Return closed polygon: apex -> base_v0 -> base_v1 -> apex
        polygon = [
            apex,
            base_vertices[0],
            base_vertices[1],
            apex  # Close the loop
        ]

        return polygon


class IntentionDomainFactory:
    """
    Factory for creating intention domain instances.

    Usage:
        domain = IntentionDomainFactory.create('cone')
        domain = IntentionDomainFactory.create('rectangle', aspect_ratio=2.0)
    """

    _registry: Dict[str, type] = {
        'cone': ConeIntentionDomain,
        # Future shapes can be registered here:
        # 'rectangle': RectangleIntentionDomain,
        # 'ellipse': EllipseIntentionDomain,
    }

    @staticmethod
    def create(domain_type: str, **kwargs) -> BaseIntentionDomain:
        """
        Create an intention domain instance by type.

        Args:
            domain_type: String identifier for domain shape ('cone', 'rectangle', etc.)
            **kwargs: Additional configuration parameters for specific domain types

        Returns:
            Instance of BaseIntentionDomain subclass

        Raises:
            ValueError: If domain_type is not registered
        """
        if domain_type not in IntentionDomainFactory._registry:
            available_types = ', '.join(IntentionDomainFactory._registry.keys())
            raise ValueError(
                f"Unknown intention domain type '{domain_type}'. "
                f"Available types: {available_types}"
            )

        domain_class = IntentionDomainFactory._registry[domain_type]
        return domain_class(**kwargs)

    @staticmethod
    def register(domain_type: str, domain_class: type) -> None:
        """
        Register a new intention domain shape.

        Args:
            domain_type: String identifier for the new shape
            domain_class: Class implementing BaseIntentionDomain
        """
        if not issubclass(domain_class, BaseIntentionDomain):
            raise TypeError(
                f"Cannot register {domain_class.__name__}: "
                f"must inherit from BaseIntentionDomain"
            )
        IntentionDomainFactory._registry[domain_type] = domain_class
