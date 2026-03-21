import copy
import numpy as np
import math

from nav_msgs.msg import OccupancyGrid


class PyCostmap2D:
    """
    PyCostmap2D.

    Costmap Python3 API for Converting Costmap2D_cpp to OccupancyGrid used for rendering.
    """

    def __init__(self, node = None):    
        self.size_x = None
        self.size_y = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.global_frame_id = "map"
        self.costmap_timestamp = None
        # Extract costmap
        self.costmap = None

        self.cost_translation_table = [0]
        for i in range(1, 253):
            self.cost_translation_table.append(1 + int(97 * (i - 1) / 251))
        self.cost_translation_table.append(99)
        self.cost_translation_table.append(100)
        self.cost_translation_table.append(-1)

        self.node = node

    def __deepcopy__(self, memo):
        # Create a new instance of PyCostmap2D
        new_copy = PyCostmap2D(self.node)
        # Copy all attributes except those that cannot be deep copied
        new_copy.size_x = self.size_x
        new_copy.size_y = self.size_y
        new_copy.resolution = self.resolution
        new_copy.origin_x = self.origin_x
        new_copy.origin_y = self.origin_y
        new_copy.global_frame_id = self.global_frame_id
        new_copy.costmap_timestamp = self.costmap_timestamp
        new_copy.costmap = copy.deepcopy(self.costmap, memo)
        return new_copy
    
    def getOccupancyGrid(self):
        """
        Load costmap from OccupancyGrid.

        Args:
        ----
            occupancy_map (OccupancyGrid): 2D OccupancyGrid Map

        """
        OccupancyGrid_msg = OccupancyGrid()
        OccupancyGrid_msg.header.frame_id = self.global_frame_id
        OccupancyGrid_msg.header.stamp = self.node.get_clock().now().to_msg()
        OccupancyGrid_msg.info.width = self.size_x
        OccupancyGrid_msg.info.height = self.size_y
        OccupancyGrid_msg.info.resolution = self.resolution
        OccupancyGrid_msg.info.origin.position.x = self.origin_x
        OccupancyGrid_msg.info.origin.position.y = self.origin_y
        OccupancyGrid_msg.info.origin.position.z = 0.0
        OccupancyGrid_msg.info.origin.orientation.x = 0.0
        OccupancyGrid_msg.info.origin.orientation.y = 0.0
        OccupancyGrid_msg.info.origin.orientation.z = 0.0
        OccupancyGrid_msg.info.origin.orientation.w = 1.0
        OccupancyGrid_msg.data = [0] * self.size_x * self.size_y

        for i in range(self.size_x * self.size_y):
            OccupancyGrid_msg.data[i] = self.cost_translation_table[self.costmap[i]]

        return OccupancyGrid_msg
    
    def loadCostmapFromCostmapCpp(self, costmap_cpp):
        """
        Load costmap from Costmap2D_cpp (create by cpp_utils).

        Args:
        ----
            costmap_cpp (Costmap2D_cpp): 2D Costmap

        """
        self.size_x = costmap_cpp.size_x
        self.size_y = costmap_cpp.size_y
        self.resolution = costmap_cpp.resolution
        self.origin_x = costmap_cpp.origin_x
        self.origin_y = costmap_cpp.origin_y
        self.costmap_timestamp = self.node.get_clock().now().to_msg()
        # Extract costmap
        self.costmap = np.array(costmap_cpp.data, dtype=np.uint8)

    def getSizeInCellsX(self):
        """Get map width in cells."""
        return self.size_x

    def getSizeInCellsY(self):
        """Get map height in cells."""
        return self.size_y

    def getSizeInMetersX(self):
        """Get x axis map size in meters."""
        return (self.size_x - 1 + 0.5) * self.resolution

    def getSizeInMetersY(self):
        """Get y axis map size in meters."""
        return (self.size_y - 1 + 0.5) * self.resolution

    def getOriginX(self):
        """Get the origin x axis of the map [m]."""
        return self.origin_x

    def getOriginY(self):
        """Get the origin y axis of the map [m]."""
        return self.origin_y

    def getResolution(self):
        """Get map resolution [m/cell]."""
        return self.resolution

    def getGlobalFrameID(self):
        """Get global frame_id."""
        return self.global_frame_id

    def getCostmapTimestamp(self):
        """Get costmap timestamp."""
        return self.costmap_timestamp

    def getCostXY(self, mx: int, my: int) -> np.uint8:
        """
        Get the cost of a cell in the costmap using map coordinate XY.

        Args
        ----
            mx (int): map coordinate X to get cost
            my (int): map coordinate Y to get cost

        Returns
        -------
            np.uint8: cost of a cell

        """
        return self.costmap[self.getIndex(mx, my)]

    def getCostIdx(self, index: int) -> np.uint8:
        """
        Get the cost of a cell in the costmap using Index.

        Args
        ----
            index (int): index of cell to get cost

        Returns
        -------
            np.uint8: cost of a cell

        """

        if index < 0 or index >= self.size_x * self.size_y:
            return 255
        else:
            return self.costmap[index]

    def setCost(self, mx: int, my: int, cost: np.uint8) -> None:
        """
        Set the cost of a cell in the costmap using map coordinate XY.

        Args
        ----
            mx (int): map coordinate X to get cost
            my (int): map coordinate Y to get cost
            cost (np.uint8): The cost to set the cell

        Returns
        -------
            None

        """
        self.costmap[self.getIndex(mx, my)] = cost

    def mapToWorld(self, mx: int, my: int) -> tuple[float, float]:
        """
        Get the world coordinate XY using map coordinate XY.

        Args
        ----
            mx (int): map coordinate X to get world coordinate
            my (int): map coordinate Y to get world coordinate

        Returns
        -------
            tuple of float: wx, wy
            wx (float) [m]: world coordinate X
            wy (float) [m]: world coordinate Y

        """
        wx = self.origin_x + (mx + 0.5) * self.resolution
        wy = self.origin_y + (my + 0.5) * self.resolution
        return (wx, wy)

    def worldToMap(self, wx: float, wy: float) -> tuple[int, int, bool]:
        """
        Get the map coordinate XY using world coordinate XY.

        Args
        ----
            wx (float) [m]: world coordinate X to get map coordinate
            wy (float) [m]: world coordinate Y to get map coordinate

        Returns
        -------
            tuple of int: mx, my
            mx (int): map coordinate X
            my (int): map coordinate Y

        """
        mx = int((wx - self.origin_x) // self.resolution)
        my = int((wy - self.origin_y) // self.resolution)
        
        is_valid = True
        if mx < 0 or mx >= self.size_x or my < 0 or my >= self.size_y:
            # raise ValueError("[PyCostmap]: The world coordinate is out of the costmap range")
            is_valid = False

        return (mx, my, is_valid)

    def getIndex(self, mx: int, my: int) -> int:
        """
        Get the index of the cell using map coordinate XY.

        Args
        ----
            mx (int): map coordinate X to get Index
            my (int): map coordinate Y to get Index

        Returns
        -------
            int: The index of the cell

        """
        return my * self.size_x + mx
    
    def set_edge_cost(self, wx0: float, wy0: float, wx1: float, wy1: float, cost_value: int):
        """
        Set the cost of the edge in the costmap.

        Args
        ----
            wx0 (float): World coordinate x0
            wy0 (float): World coordinate y0
            wx1 (float): World coordinate x1
            wy1 (float): World coordinate y1
            cost_value (int): Cost value to set

        """
        direction_vector = [(wx1 - wx0) / math.sqrt((wx1 - wx0) ** 2 + (wy1 - wy0) ** 2),
                            (wy1 - wy0) / math.sqrt((wx1 - wx0) ** 2 + (wy1 - wy0) ** 2)]
        distance = 0
        module = math.sqrt((wx1 - wx0) ** 2 + (wy1 - wy0) ** 2)
        warn_flag = False

        while distance <= module:
            mx, my, is_valid = self.worldToMap(wx0 + distance * direction_vector[0],
                                        wy0 + distance * direction_vector[1])
            if is_valid:
                self.setCost(mx, my, cost_value)
            else:
                warn_flag = True
                
            distance += self.resolution

        if warn_flag:
            pass
            # print("[Path Planner] The edge of cone is out of range.")

    def load_domain_to_map(
        self,
        intention_domain_obj,
        inflated_distance: float
    ):
        """
        Load intention domain as obstacles onto the costmap using bounding box scan.

        This method implements the "Bounding Box Scan" algorithm to ensure perfect
        alignment between Python and C++ rendering:
        1. Get physical bounding box from intention domain
        2. Convert to grid index bounds
        3. Scan all cells within bounding box
        4. Query spatial predicate is_restricted_area() for each cell
        5. Mark restricted cells as LETHAL_OBSTACLE (254)

        This approach eliminates diagonal gaps present in line-drawing methods and
        supports arbitrary curved shapes (e.g., ellipses).

        Args
        ----
            intention_domain_obj: Instance of BaseIntentionDomain (e.g., ConeIntentionDomain)
            action_params (list[float]): Shape-specific parameters (e.g., [depth, radius] for cone)
            cur_pos (list[float]): Robot's current position [x, y]
            robot_direction (np.ndarray): Normalized direction vector [dx, dy]
            inflated_distance (float): Inflation margin for robot safety

        Returns
        -------
            PyCostmap2D: Deep copy of costmap with domain obstacles marked

        """
        # Step 1: Deep copy the base costmap
        map_with_domain = copy.deepcopy(self)

        # Step 2: Get physical bounding box in world coordinates
        min_x, max_x, min_y, max_y = intention_domain_obj.get_bounding_box(
            inflated_distance=inflated_distance
        )

        # Step 3: Convert world bounding box to grid index bounds (with clipping)
        min_mx, min_my, _ = map_with_domain.worldToMap(min_x, min_y)
        max_mx, max_my, _ = map_with_domain.worldToMap(max_x, max_y)

        # Clamp to valid map range
        min_mx = max(0, min(min_mx, self.size_x - 1))
        max_mx = max(0, min(max_mx, self.size_x - 1))
        min_my = max(0, min(min_my, self.size_y - 1))
        max_my = max(0, min(max_my, self.size_y - 1))

        # Step 4: Bounding box scan - iterate over all cells in the box
        for mx in range(min_mx, max_mx + 1):
            for my in range(min_my, max_my + 1):
                # Step 5: Convert grid cell center back to world coordinates
                wx, wy = map_with_domain.mapToWorld(mx, my)

                # Step 6: Query spatial predicate
                if not intention_domain_obj.is_restricted_area(
                    wx=wx,
                    wy=wy,
                ):
                    # Mark as LETHAL_OBSTACLE (same as C++ LETHAL_OBSTACLE = 254)
                    map_with_domain.setCost(mx, my, 254)

        return map_with_domain
