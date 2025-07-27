"""
功能：
pgm+yaml转gazebo 3D地图

依赖：
pip install opencv-python pyyaml numpy

示例:
python3 /home/rosdev/ros2_ws/rl_replanner_train/maps/pgm_to_gazebo.py   --map /home/rosdev/ros2_ws/rl_replanner_train/maps/square_map/square_map_2.yaml   --output /home/rosdev/ros2_ws/rl_replanner_train/maps/gazebo_map/square_map_2.world
"""

import cv2
import yaml
import argparse
import numpy as np
import os

def find_obstacle_rectangles(map_data):
    h, w = map_data.shape
    visited = np.zeros_like(map_data, dtype=bool)
    rectangles = []

    for r in range(h):
        for c in range(w):
            if map_data[r, c] == 0 and not visited[r, c]:
                max_w = w
                # 1. 计算最大宽度
                for i in range(c, w):
                    if map_data[r, i] != 0 or visited[r, i]:
                        max_w = i - c
                        break
                else:
                    max_w = w - c

                # 2. 基于此宽度计算最大高度
                max_h = h
                for j in range(r, h):
                    for i in range(c, c + max_w):
                        if map_data[j, i] != 0 or visited[j, i]:
                            max_h = j - r
                            break
                    if max_h != h:
                        break
                else:
                    max_h = h - r
                
                visited[r:r+max_h, c:c+max_w] = True
                rectangles.append((c, r, max_w, max_h))
    return rectangles

def generate_gazebo_world(map_yaml_path, output_world_path, obstacle_height=1.0):
    # 1. 解析YAML文件
    with open(map_yaml_path, 'r') as f:
        try:
            map_config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file: {e}")
            return

    resolution = map_config['resolution']
    origin_x, origin_y, _ = map_config['origin']
    pgm_file_name = map_config['image']
    
    yaml_dir = os.path.dirname(map_yaml_path)
    pgm_path = os.path.join(yaml_dir, pgm_file_name)

    if not os.path.exists(pgm_path):
        print(f"Error: PGM file not found at '{pgm_path}'")
        return

    # 2. 读取PGM文件
    map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if map_image is None:
        print(f"Error: Could not read PGM file '{pgm_path}'")
        return
        
    map_height_pixels, map_width_pixels = map_image.shape
    print(f"Map loaded: {map_width_pixels}x{map_height_pixels} pixels, Resolution: {resolution} m/px")

    # 3. 查找障碍物矩形
    print("Finding and merging obstacle pixels...")
    rectangles = find_obstacle_rectangles(map_image)
    print(f"Found {len(rectangles)} merged obstacle rectangles.")

    # 4. 生成 .world 文件内容
    sdf_models = ""
    for i, (px, py, pw, ph) in enumerate(rectangles):
        box_width = pw * resolution
        box_depth = ph * resolution

        center_x = origin_x + (px + pw / 2.0) * resolution
        center_y = origin_y + (map_height_pixels - (py + ph / 2.0)) * resolution
        
        sdf_models += f"""
    <model name='obstacle_{i}'>
      <static>true</static>
      <pose>{center_x} {center_y} {obstacle_height / 2.0} 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{box_width} {box_depth} {obstacle_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{box_width} {box_depth} {obstacle_height}</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>"""

    world_template = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    {sdf_models}
  </world>
</sdf>
"""
    # 5. 写入文件
    with open(output_world_path, 'w') as f:
        f.write(world_template)
    
    print(f"Gazebo world file successfully generated at: {output_world_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a ROS 2D map (PGM+YAML) to a Gazebo 3D world.")
    parser.add_argument('--map', type=str, required=True,
                        help='Path to the input map.yaml file.')
    parser.add_argument('--output', type=str, required=True,
                        help='Path to the output .world file.')
    parser.add_argument('--height', type=float, default=1.0,
                        help='Height of the obstacles in Gazebo (in meters).')
    
    args = parser.parse_args()
    
    generate_gazebo_world(args.map, args.output, args.height)
