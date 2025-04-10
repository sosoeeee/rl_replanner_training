import cpp_utils
import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

rclpy.init()

render_node = rclpy.create_node("render_node")

path_publisher = render_node.create_publisher(Path, "path", 10)
obstacles_publisher = render_node.create_publisher(MarkerArray, "obstacles", 10)

# load config
tebConfig = cpp_utils.TebConfig()
tebConfig.configure(yaml_filename = "/home/rosdev/ros2_ws/cpp_utils/include/teb_local_planner/teb_params.yaml")

# load obstacles
obstacles = []
# circular_obstacles_params = [
#     (2.5, 0.4, 0.2)
# ]
# marker_array = MarkerArray()

# for x, y, r in circular_obstacles_params:
#     obstacles.append(cpp_utils.CircularObstacle(x, y, r))
#     marker = Marker()
#     marker.header.frame_id = "map"
#     marker.header.stamp = render_node.get_clock().now().to_msg()
#     # marker.ns = "obstacles"
#     marker.id = len(marker_array.markers)
#     marker.type = Marker.SPHERE
#     marker.action = Marker.ADD
#     marker.pose.position.x = x
#     marker.pose.position.y = y
#     marker.pose.position.z = 0.0
#     marker.scale.x = r * 2
#     marker.scale.y = r * 2
#     marker.scale.z = 0.1
#     marker.color.r = 1.0
#     marker.color.g = 0.0
#     marker.color.b = 0.0
#     marker.color.a = 1.0
#     # marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
#     marker_array.markers.append(marker)

# load obstacles
circular_corridor_params = [
    (0.0, 0.0, 1.0),
    (1.5, 0.0, 1.0),
    (2.5, 1.0, 0.75),
    (4.0, 2.0, 1.5)
]
corridor = cpp_utils.CircularCorridor()
marker_array = MarkerArray()
for x, y, r in circular_corridor_params:
    corridor.addCircle(x, y, r)
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = render_node.get_clock().now().to_msg()
    # marker.ns = "obstacles"
    marker.id = len(marker_array.markers)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0
    marker.scale.x = r * 2
    marker.scale.y = r * 2
    marker.scale.z = 0.1
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.2
    # marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
    marker_array.markers.append(marker)
obstacles.append(corridor)

# load viapoints
viaPoints_params = [
    (1.5, -0.75)
]
viaPoints = cpp_utils.ViaPointContainer()

for x, y in viaPoints_params:
    viaPoints.push_back(x, y)
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = render_node.get_clock().now().to_msg()
    # marker.ns = "via_points"
    marker.id = len(marker_array.markers)
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 0.5
    # marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
    marker_array.markers.append(marker)

# teb planner
tebPlanner = cpp_utils.TebOptimalPlanner()
tebPlanner.initialize(tebConfig, obstacles, viaPoints)
startPoint = cpp_utils.PoseSE2(0.0, 0.0, np.pi)
endPoint = cpp_utils.PoseSE2(4.0, 2.0, np.pi)

while rclpy.ok():
    tebPlanner.plan(startPoint, endPoint)
    # get path
    traj = tebPlanner.getFullTrajectory()

    # convert path to Path message
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = render_node.get_clock().now().to_msg()

    time_stamps = []
    x_vels = []
    y_vels = []
    omage_vels = []
    x_pos = []
    y_pos = []
    that_pos = []
    for point in traj:
        pose = PoseStamped()
        pose.pose.position.x = point.pose.x
        pose.pose.position.y = point.pose.y
        pose.pose.position.z = 0.0
        path_msg.poses.append(pose)

        time_stamps.append(point.time_from_start)
        x_vels.append(point.velocity.vx)
        y_vels.append(point.velocity.vy)
        omage_vels.append(point.velocity.omega)
        x_pos.append(point.pose.x)
        y_pos.append(point.pose.y)
        that_pos.append(point.pose.theta)

    path_publisher.publish(path_msg)
    obstacles_publisher.publish(marker_array)
    rclpy.spin_once(render_node, timeout_sec=0.1)

    # plot the trajectory
    # plt.figure(1)
    # plt.clf()
    # plt.subplot(211)
    # plt.plot(time_stamps, x_vels, label="x velocity")
    # plt.plot(time_stamps, y_vels, label="y velocity")
    # plt.plot(time_stamps, omage_vels, label="omega velocity")

    # plt.xlabel("time [s]")
    # plt.ylabel("velocity [m/s]")
    # plt.legend()
    # plt.grid()
    # plt.subplot(212)
    # plt.plot(time_stamps, x_pos, label="x position")
    # plt.plot(time_stamps, y_pos, label="y position")
    # plt.plot(time_stamps, that_pos, label="theta position")
    # plt.xlabel("time [s]")
    # plt.ylabel("position [m]")
    # plt.legend()
    # plt.grid()
    # plt.pause(0.01)


# rclpy.shutdown()
