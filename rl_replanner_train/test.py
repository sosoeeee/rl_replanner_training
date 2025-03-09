import cpp_utils
import rclpy
from maps.costmap_2d import PyCostmap2D

from nav_msgs.msg import OccupancyGrid

rclpy.init()

render_node = rclpy.create_node("render_node")

publisher = render_node.create_publisher(OccupancyGrid, "costmap", 10)

res_status, res_costmap = cpp_utils.loadMap("/home/rosdev/ros2_ws/rl_replanner_train/maps/tb3_classic/turtlebot3_world.yaml")

if res_status == cpp_utils.LOAD_MAP_STATUS.LOAD_MAP_SUCCESS:
    pyCostmap = PyCostmap2D(render_node)
    pyCostmap.loadCostmapFromCostmapCpp(res_costmap)

    while rclpy.ok():
        publisher.publish(pyCostmap.getOccupancyGrid())
        rclpy.spin_once(render_node, timeout_sec=0.1)

# rclpy.shutdown()
