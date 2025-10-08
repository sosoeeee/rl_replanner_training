import cpp_utils
import os

def get_map_and_config_paths(base_path):
    map_yaml_path = os.path.join(base_path, '..', 'data_collection', 'maps', 'map.yaml')

    planner_config_path = os.path.join(base_path, '..', 'data_collection', 'collector_params.yaml')

    if not os.path.exists(map_yaml_path):
        print(f"Warning: Map file not found at assumed path: {map_yaml_path}")
        fallback_map_path = os.path.join(base_path, '..', 'data_collection', 'maps')
        if os.path.exists(fallback_map_path):
            for file in os.listdir(fallback_map_path):
                if file.endswith('.yaml'):
                    map_yaml_path = os.path.join(fallback_map_path, file)
                    print(f"Found a map file at: {map_yaml_path}")
                    break
    
    if not os.path.exists(planner_config_path):
         print(f"Warning: Planner config not found at assumed path: {planner_config_path}")

    return map_yaml_path, planner_config_path


def main():
    waypoints = [
        cpp_utils.Point(-4.0, -4.0),
        # cpp_utils.Point(-1.6, 0.0),
        # cpp_utils.Point(-3.0, 1.4),
        # cpp_utils.Point(-4.0, -0.4),
        # cpp_utils.Point(3.4, -3.8),
        # cpp_utils.Point(4.0, -2.0),
        # cpp_utils.Point(-0.4, 1.6),
        cpp_utils.Point(4.0, 4.0),
    ]

    output_filename = "marked_path.txt"

    map_yaml = "/home/rosdev/ros2_ws/train_env/rl_replanner_training/rl_replanner_train/maps/sim_maps/circle_clutter.yaml"
    planner_config_yaml = "/home/rosdev/ros2_ws/train_env/rl_replanner_training/rl_replanner_train/gennerate_marked_paths/planner_config.yaml"

    if not os.path.exists(map_yaml):
        print(f"错误: 地图配置文件未找到: {map_yaml}")
        return
    if not os.path.exists(planner_config_yaml):
        print(f"警告: 规划器配置文件未找到: {planner_config_yaml}")

    print(f"正在从 {map_yaml} 加载地图...")
    status, costmap = cpp_utils.loadMap(map_yaml)
    if status != cpp_utils.LOAD_MAP_STATUS.LOAD_MAP_SUCCESS:
        print(f"错误: 无法从 {map_yaml} 加载地图。状态码: {status}")
        return

    print("正在初始化和配置路径规划器...")
    planner = cpp_utils.PathPlanner()
    planner.configure(costmap, planner_config_yaml)
    print("规划器配置成功。")

    full_path = []
    print(f"Starting to plan path through {len(waypoints)} waypoints.")

    for i in range(len(waypoints) - 1):
        start_point = waypoints[i]
        goal_point = waypoints[i+1]
        
        print(f"Planning segment {i+1}: from ({start_point.x:.2f}, {start_point.y:.2f}) to ({goal_point.x:.2f}, {goal_point.y:.2f})")

        path_segment = planner.plan(start_point, goal_point)

        if not path_segment:
            print(f"Error: Failed to find a path for segment {i+1}. Aborting.")
            return
        
        print(f"Segment {i+1} planned successfully with {len(path_segment)} points.")

        if i > 0:
            path_segment.pop(0)
            
        full_path.extend(path_segment)

    if not full_path:
        print("No path was generated.")
        return

    print(f"\nTotal path generated with {len(full_path)} points.")

    try:
        with open(output_filename, 'w') as f:
            for point in full_path:
                f.write(f"{point.x} {point.y}\n")
        print(f"Successfully saved the complete path to '{output_filename}'")
    except IOError as e:
        print(f"Error: Failed to write to file '{output_filename}'. Reason: {e}")


if __name__ == "__main__":
    main()
