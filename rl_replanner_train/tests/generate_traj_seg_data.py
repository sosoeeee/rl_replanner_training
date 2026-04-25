import cpp_utils
import numpy as np
import rclpy
import time
import os
from rl_replanner_train.render.costmap_2d import PyCostmap2D
from tqdm import tqdm

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid

# import matplotlib.pyplot as plt
# import matplotlib
# matplotlib.use('TkAgg')

map_name = "phy1"
init_pose = (-3.08, 0.57)
target_pose = (0.789, 3.348)
start_point = cpp_utils.Point(init_pose[0], init_pose[1])
end_point = cpp_utils.Point(target_pose[0], target_pose[1])

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(current_dir))
map_file = os.path.join(project_root, "rl_replanner_train", "maps", "real_maps", "phy1.yaml")
planner_file = os.path.join(project_root, "cpp_utils", "include", "teb_local_planner", "teb_params.yaml")

rclpy.init()

render_node = rclpy.create_node("render_node")

path_publisher_past = render_node.create_publisher(Path, "path_past", 10)
path_publisher_future = render_node.create_publisher(Path, "path_future", 10)
costmap_publisher = render_node.create_publisher(OccupancyGrid, "costmap", 10)

previous_marker_count = 0  # Add this line after creating render_node

res_status, costmap_cpp = cpp_utils.loadMap(map_file)
pyCostmap = PyCostmap2D(render_node)

# Initialize the trajectory generator
traj_generator = cpp_utils.TrajGenerator()
traj_generator.initialize(
    map_file=map_file,
    planner_file=planner_file,
    path_resolution=0.025,
    time_resolution=0.1,
)

print("Initialized trajectory generator")

pyCostmap.loadCostmapFromCostmapCpp(traj_generator.getCostmap())
# startPoint = cpp_utils.Point(-1.72, -0.217)
# endPoint = cpp_utils.Point(1.96, 0.395)

# phy1
# startPoint = cpp_utils.Point(-2.42, 4.77)
# endPoint = cpp_utils.Point(-5.57,  8.41)

# World to Map
# startPoint_map = pyCostmap.worldToMap(-1.72, -0.217)
# endPoint_map = pyCostmap.worldToMap(1.96, 0.395)
# print("startPoint_map: ", startPoint_map)
# print("endPoint_map: ", endPoint_map)

# predicted data format
past_time_frames = 20
future_time_frames = 40
total_time_frames = past_time_frames + future_time_frames
obser_width = 5.0
MAX_SAMPLES = 1000
partial_map = costmap_cpp.getPartialCostmap(start_point.x, start_point.y, obser_width, obser_width)
size_x = partial_map.size_x
size_y = partial_map.size_y
traj_array = np.zeros((MAX_SAMPLES, 1, total_time_frames, 2))  # (N_batches, N_agents, N_time_frames, 2)
traj_array_abs = np.zeros((MAX_SAMPLES, 1, total_time_frames, 2))  # absolute positions in world frame
map_array = np.zeros((MAX_SAMPLES, size_x, size_y))  # (N_batches, height, width)
global_goal = np.array([end_point.x, end_point.y]) 
idx = 0

# Initialize progress bar
with tqdm(total=MAX_SAMPLES, desc="Collecting Data", unit="sample") as pbar:
    # for episode in range(N_episodes):
    while idx < MAX_SAMPLES:
        # Use the new function that returns two trajectories
        traj = traj_generator.sampleTrajLoop(start=start_point, end=end_point)

        # cut to segments
        for i in range(0, len(traj) - total_time_frames + 1, total_time_frames // 2):
            traj_segment = traj[i:i + total_time_frames]
            cur_pos = traj_segment[past_time_frames - 1]  # current position at the end of past frames

            # draw traj to partial map
            temp_costmap_cpp = costmap_cpp.copy()
            for i in range(past_time_frames):
                mx, my, _ = pyCostmap.worldToMap(traj_segment[i].x, traj_segment[i].y)
                temp_costmap_cpp.setCost(mx, my, 200)  # set the cost

            # draw global goal to partial map
            if abs(global_goal[0] - cur_pos.x) < obser_width / 2 and abs(global_goal[1] - cur_pos.y) < obser_width / 2:
                mx, my, _ = pyCostmap.worldToMap(global_goal[0], global_goal[1])
                # inflate the cost around the intersection point
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        temp_costmap_cpp.setCost(mx + dx, my + dy, 200)  # set the cost
            else:
                # find the intersection point of the line from current position to global goal and the boundary of the partial map
                dx = global_goal[0] - cur_pos.x
                dy = global_goal[1] - cur_pos.y
                scale = min(obser_width / 2 / abs(dx), obser_width / 2 / abs(dy))
                intersection_x = cur_pos.x + dx * scale
                intersection_y = cur_pos.y + dy * scale
                mx, my, _ = pyCostmap.worldToMap(intersection_x, intersection_y)
                mx -= 1
                my -= 1
                # inflate the cost around the intersection point
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        temp_costmap_cpp.setCost(mx + dx, my + dy, 200)  # set the cost

            partial_map = temp_costmap_cpp.getPartialCostmap(cur_pos.x, cur_pos.y, obser_width, obser_width)
            data = np.array(partial_map.data, dtype=np.uint8)
            partial_map_2d = data.reshape((partial_map.size_y, partial_map.size_x))

            # Convert the trajectory segment to a numpy array
            traj_segment_array_abs = np.array([[point.x, point.y] for point in traj_segment])
            traj_segment_array_rela = np.array([[point.x - cur_pos.x, point.y - cur_pos.y] for point in traj_segment])  # relative positions
            traj_array_abs[idx, 0, :total_time_frames, :] = traj_segment_array_abs
            traj_array[idx, 0, :total_time_frames, :] = traj_segment_array_rela
            map_array[idx, :, :] = partial_map_2d
            idx += 1

            if idx >= MAX_SAMPLES:
                break

            # Update progress bar
            pbar.update(1)

            # DEBUG: visualize the trajectory and the costmap
            # path_msg_past = Path()
            # path_msg_past.header.frame_id = "map"
            # path_msg_past.header.stamp = render_node.get_clock().now().to_msg()

            # for point in traj_segment[:past_time_frames]:
            #     pose = PoseStamped()
            #     pose.pose.position.x = point.x
            #     pose.pose.position.y = point.y
            #     pose.pose.position.z = 0.0
            #     path_msg_past.poses.append(pose)
            
            # path_msg_future = Path()
            # path_msg_future.header.frame_id = "map"
            # path_msg_future.header.stamp = render_node.get_clock().now().to_msg()

            # for point in traj_segment[past_time_frames:]:
            #     pose = PoseStamped()
            #     pose.pose.position.x = point.x
            #     pose.pose.position.y = point.y
            #     pose.pose.position.z = 0.0
            #     path_msg_future.poses.append(pose)

            # path_publisher_past.publish(path_msg_past)
            # path_publisher_future.publish(path_msg_future)

            # temp_py_costmap = PyCostmap2D(render_node)
            # temp_py_costmap.loadCostmapFromCostmapCpp(partial_map)
            # occ = temp_py_costmap.getOccupancyGrid()
            # costmap_publisher.publish(occ)

            # rclpy.spin_once(render_node, timeout_sec=0.1)

            # time.sleep(1)  # sleep for a while to visualize

# save data as npy files
np.save(os.path.join(project_root, "rl_replanner_train", "data", f"traj_array_{map_name}.npy"), traj_array[:idx])
np.save(os.path.join(project_root, "rl_replanner_train", "data", f"traj_array_abs_{map_name}.npy"), traj_array_abs[:idx])
np.save(os.path.join(project_root, "rl_replanner_train", "data", f"map_array_{map_name}.npy"), map_array[:idx])
print(f"Saved {idx} samples to npy files.")
