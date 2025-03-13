import cpp_utils
import rclpy
from maps.costmap_2d import PyCostmap2D

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rclpy.init()

render_node = rclpy.create_node("render_node")

costmap_publisher = render_node.create_publisher(OccupancyGrid, "costmap", 10)
partial_map_publisher = render_node.create_publisher(OccupancyGrid, "partial_map", 10)
path_publisher = render_node.create_publisher(Path, "path", 10)

res_status, costmap_cpp = cpp_utils.loadMap("/home/rosdev/ros2_ws/rl_replanner_train/maps/tb3_classic/turtlebot3_world.yaml")

if res_status == cpp_utils.LOAD_MAP_STATUS.LOAD_MAP_SUCCESS:
    pyCostmap = PyCostmap2D(render_node)
    pyCostmap.loadCostmapFromCostmapCpp(costmap_cpp)
    
    # goal_mx, goal_my = pyCostmap.worldToMap(1.96, 0.395)
    # start_mx, start_my = pyCostmap.worldToMap(-1.72, -0.217)
    # print("Goal: ", goal_mx, goal_my)
    # print("Start: ", start_mx, start_my)
    # print("Start cost: ", pyCostmap.getCostIdx(pyCostmap.getIndex(start_mx, start_my)))
    # print("Goal cost: ", pyCostmap.getCostIdx(pyCostmap.getIndex(goal_mx, goal_my)))

    pathPlanner = cpp_utils.PathPlanner()
    pathPlanner.configure(costmap_cpp, "/home/rosdev/ros2_ws/cpp_utils/include/path_planner/planner_setting.yaml")
    startPoint = cpp_utils.Point(-1.72, -0.217)
    endPoint = cpp_utils.Point(1.96, 0.395)
    path = pathPlanner.plan(startPoint, endPoint)

    partial_map = costmap_cpp.getPartialCostmap(-1.72, -0.217, 3.0, 3.0)
    pyCostmap_partial = PyCostmap2D(render_node)
    pyCostmap_partial.loadCostmapFromCostmapCpp(partial_map)

    # convert path to Path message
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = render_node.get_clock().now().to_msg()
    for point in path:
        pose = PoseStamped()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = 0.0
        path_msg.poses.append(pose)

    while rclpy.ok():
        costmap_publisher.publish(pyCostmap.getOccupancyGrid())
        partial_map_publisher.publish(pyCostmap_partial.getOccupancyGrid())
        path_publisher.publish(path_msg)
        rclpy.spin_once(render_node, timeout_sec=0.1)

# rclpy.shutdown()
