import os
import cpp_utils
import numpy as np
import rclpy
import time
from rl_replanner_train.render.costmap_2d import PyCostmap2D

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

rclpy.init()

render_node = rclpy.create_node("render_node")

path_publisher = render_node.create_publisher(Path, "path", 10)
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

# 轨迹文件
data_dir = "rl_replanner_train/data/eval_paths"
if not os.path.exists(data_dir):
    print("data_dir not exists")
    exit()

path_index = 0  # 文件名索引

while rclpy.ok():
    start_time = time.time()
    traj = traj_generator.sampleDistinctHomotopyTrajs(start=startPoint, end=endPoint)
    end_time = time.time()

    if len(traj) == 0:
        print("Finish sampling all trajectories")
        break

    print("Time taken to sample trajectory: %.2f ms" % ((end_time - start_time) * 1000))

    # 计算差分速度
    path_data = []
    for i in range(len(traj) - 1):
        x, y = traj[i].x, traj[i].y
        next_x, next_y = traj[i + 1].x, traj[i + 1].y
        dx = (next_x - x) / 0.1
        dy = (next_y - y) / 0.1
        theta = 0
        dtheta = 0
        t = i * 0.1
        path_data.append([x, y, theta, dx, dy, dtheta, t])

    # 保存
    file_path = os.path.join(data_dir, f"path_{path_index}.txt")
    np.savetxt(file_path, path_data, fmt='%.18e', delimiter=' ')
    print(f"Path saved to {file_path}")
    path_index += 1

    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = render_node.get_clock().now().to_msg()

    idx = 0
    for point in traj:
        pose = PoseStamped()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = 0.0
        path_msg.poses.append(pose)
        idx += 1
        
    print("total number of points: ", idx)

    path_publisher.publish(path_msg)
    costmap_publisher.publish(pyCostmap.getOccupancyGrid()) 

    rclpy.spin_once(render_node, timeout_sec=0.1)
