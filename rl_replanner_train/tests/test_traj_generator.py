import cpp_utils
import numpy as np
import rclpy
import time
from rl_replanner_train.render.costmap_2d import PyCostmap2D

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid

# import matplotlib.pyplot as plt
# import matplotlib
# matplotlib.use('TkAgg')

rclpy.init()

render_node = rclpy.create_node("render_node")

path_publisher = render_node.create_publisher(Path, "path", 10)
init_path_publisher = render_node.create_publisher(Path, "init_path", 10)
raw_traj_publisher = render_node.create_publisher(Path, "raw_traj", 10)
marker_publisher = render_node.create_publisher(MarkerArray, "obstacles", 10)
costmap_publisher = render_node.create_publisher(OccupancyGrid, "costmap", 10)

previous_marker_count = 0  # Add this line after creating render_node

res_status, costmap_cpp = cpp_utils.loadMap("/home/rosdev/ros2_ws/rl_replanner_train/maps/tb3_classic/turtlebot3_world.yaml")
pyCostmap = PyCostmap2D(render_node)


# Initialize the trajectory generator
traj_generator = cpp_utils.TrajGenerator()
traj_generator.initialize(
    map_file="/home/rosdev/ros2_ws/rl_replanner_train/maps/tb3_classic/turtlebot3_world.yaml",
    planner_file="/home/rosdev/ros2_ws/cpp_utils/include/teb_local_planner/teb_params.yaml",
    path_resolution=0.025,
    time_resolution=0.1,
)
pyCostmap.loadCostmapFromCostmapCpp(traj_generator.getCostmap())
startPoint = cpp_utils.Point(-1.72, -0.217)
endPoint = cpp_utils.Point(1.96, 0.395)


while rclpy.ok():
    traj = traj_generator.sampleTraj(start = startPoint, end = endPoint)

    print("Finish sampling trajectory")

    init_path = traj_generator.getInitPlan()
    corridor = traj_generator.getCircles()
    viaPoints = traj_generator.getViaPoints()
    rawTraj = traj_generator.getRawTraj()

    # convert path to Path message
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = render_node.get_clock().now().to_msg()

    last_point = None
    idx = 0
    for point in traj:
        if last_point is not None:
            distance = np.linalg.norm(np.array([point.x, point.y]) - np.array([last_point.x, last_point.y]))
            # print("Distance between points: ", distance)
            if distance > 0.1:
                print("Distance between points: ", distance)
                print("point: ", round(point.x, 6), round(point.y, 6), "idx: ", idx + 1)
                print("last_point: ", round(last_point.x, 6), round(last_point.y, 6), "idx: ", idx)
        last_point = point
        pose = PoseStamped()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = 0.0
        path_msg.poses.append(pose)
        idx += 1
    
    print("total number of points: ", idx)
    # for point in traj:
    #     pose = PoseStamped()
    #     pose.pose.position.x = point.x
    #     pose.pose.position.y = point.y
    #     pose.pose.position.z = 0.0
    #     path_msg.poses.append(pose)

    init_path_msg = Path()
    init_path_msg.header.frame_id = "map"
    init_path_msg.header.stamp = render_node.get_clock().now().to_msg()

    for point in init_path:
        pose = PoseStamped()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = 0.0
        init_path_msg.poses.append(pose)

    raw_traj_msg = Path()
    raw_traj_msg.header.frame_id = "map"
    raw_traj_msg.header.stamp = render_node.get_clock().now().to_msg()

    for point in rawTraj:
        pose = PoseStamped()
        pose.pose.position.x = point.pose.x
        pose.pose.position.y = point.pose.y
        pose.pose.position.z = 0.0
        raw_traj_msg.poses.append(pose)

    # First, delete all previous markers
    markers = MarkerArray()
    for i in range(previous_marker_count):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = render_node.get_clock().now().to_msg()
        marker.id = i
        marker.action = Marker.DELETE
        markers.markers.append(marker)
    
    if len(markers.markers) > 0:
        marker_publisher.publish(markers)
        time.sleep(0.1)  # Give time for deletion to process
    
    # Now create new markers
    markers = MarkerArray()
    marker_id = 0

    # Add corridor circles
    for circle in corridor:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = render_node.get_clock().now().to_msg()
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = circle.x
        marker.pose.position.y = circle.y
        marker.pose.position.z = 0.0
        marker.scale.x = circle.radius * 2
        marker.scale.y = circle.radius * 2
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.2
        markers.markers.append(marker)

    # Add via points
    for point in viaPoints:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = render_node.get_clock().now().to_msg()
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        markers.markers.append(marker)

    # Update the previous marker count for next iteration
    previous_marker_count = marker_id
    
    # Publish new markers
    marker_publisher.publish(markers)
        
    costmap_publisher.publish(pyCostmap.getOccupancyGrid()) 
    path_publisher.publish(path_msg)
    init_path_publisher.publish(init_path_msg)
    raw_traj_publisher.publish(raw_traj_msg)

    rclpy.spin_once(render_node, timeout_sec=0.1)

    # sleep for 1 second
    time.sleep(1.0)

# rclpy.shutdown()
