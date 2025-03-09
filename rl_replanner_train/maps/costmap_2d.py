import numpy as np

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

    def worldToMap(self, wx: float, wy: float) -> tuple[int, int]:
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

        if mx < 0 or mx >= self.size_x or my < 0 or my >= self.size_y:
            raise ValueError("[PyCostmap]: The world coordinate is out of the costmap range")

        return (mx, my)

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

    def getPartialCostmap(self, wx: float, wy: float, w_size_x: float, w_size_y: float) -> np.array:
        """
        Get the partial costmap using world coordinate XY and size.

        Args
        ----
            the center of the partial costmap is (wx, wy)
            wx (float) [m]: world coordinate X
            wy (float) [m]: world coordinate Y
            w_size_x (float) [m]: width of the partial costmap
            w_size_y (float) [m]: height of the partial costmap

        Returns
        -------
            nav_msgs.msg.OccupancyGrid: The partial costmap

        """
        mx, my = self.worldToMap(wx, wy)
        m_size_x = int(w_size_x / self.resolution)
        m_size_y = int(w_size_y / self.resolution)
        start_x = mx - m_size_x // 2
        start_y = my - m_size_y // 2
        
        # Get the starting index of the partial costmap
        partial_costmap = np.array([self.getCostIdx(self.getIndex(start_x + i, start_y + j)) for j in range(m_size_y) for i in range(m_size_x)])

        partial_costmap_msg = OccupancyGrid()
        partial_costmap_msg.header.frame_id = self.global_frame_id
        partial_costmap_msg.header.stamp = self.node.get_clock().now().to_msg()
        partial_costmap_msg.info.width = m_size_x
        partial_costmap_msg.info.height = m_size_y
        partial_costmap_msg.info.resolution = self.resolution
        partial_costmap_msg.info.origin.position.x = self.origin_x + start_x * self.resolution
        partial_costmap_msg.info.origin.position.y = self.origin_y + start_y * self.resolution
        partial_costmap_msg.info.origin.position.z = 0.0
        partial_costmap_msg.info.origin.orientation.x = 0.0
        partial_costmap_msg.info.origin.orientation.y = 0.0
        partial_costmap_msg.info.origin.orientation.z = 0.0
        partial_costmap_msg.info.origin.orientation.w = 1.0
        partial_costmap_msg.data = [0] * m_size_x * m_size_y

        for i in range(m_size_x * m_size_y):
            partial_costmap_msg.data[i] = self.cost_translation_table[partial_costmap[i]]

        return partial_costmap_msg