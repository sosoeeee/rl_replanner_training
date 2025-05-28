import cpp_utils
import numpy as np
import rclpy
import time
from rl_replanner_train.render.costmap_2d import PyCostmap2D

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid

# import matplotlib.pyplot as plt
# import matplotlib
# matplotlib.use('TkAgg')

rclpy.init()

render_node = rclpy.create_node("render_node")

path_publisher = render_node.create_publisher(Path, "path", 10)
costmap_publisher = render_node.create_publisher(OccupancyGrid, "costmap", 10)

previous_marker_count = 0  # Add this line after creating render_node

map_name = "turtlebot3_world_3"

res_status, costmap_cpp = cpp_utils.loadMap("./rl_replanner_train/maps/tb3_classic/" + map_name + ".yaml")
pyCostmap = PyCostmap2D(render_node)


# Initialize the trajectory generator
traj_generator = cpp_utils.TrajGenerator()
traj_generator.initialize(
    map_file="./rl_replanner_train/maps/tb3_classic/" + map_name + ".yaml",
    planner_file="./cpp_utils/include/teb_local_planner/teb_params.yaml",
    path_resolution=0.025,
    time_resolution=0.1,
)
pyCostmap.loadCostmapFromCostmapCpp(traj_generator.getCostmap())
startPoint = cpp_utils.Point(-1.72, -0.217)
endPoint = cpp_utils.Point(1.96, 0.395)

while rclpy.ok():
    start_time = time.time()
    traj = traj_generator.sampleDistinctHomotopyTrajs(start=startPoint, end=endPoint)
    end_time = time.time()

    print("Time taken to sample trajectories: %.2f ms" % ((end_time - start_time) * 1000))

    # Print all the trajectories
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
    # for point in traj:
    #     pose = PoseStamped()
    #     pose.pose.position.x = point.x
    #     pose.pose.position.y = point.y
    #     pose.pose.position.z = 0.0
    #     path_msg.poses.append(pose)

    path_publisher.publish(path_msg)
    costmap_publisher.publish(pyCostmap.getOccupancyGrid()) 

    rclpy.spin_once(render_node, timeout_sec=0.1)

    if (len(traj) == 0):
        print("Finish sampling all trajectories")
        time.sleep(1)
    else:
        # sleep for 1 second
        # time.sleep(0.5)
        pass

# rclpy.shutdown()
