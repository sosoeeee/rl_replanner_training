'''
直接生成pgm+yaml文件并生成Gazebo world文件

示例：
python3 circle_map_to_gazebo_generator.py \
    --size 22 --obstacles 10 --boundary 1.0 \
    --output ./circle_map/circle_map_2 \
    --world_output ./gazebo_map/circle_map_2.world
'''

import numpy as np
import cv2
import yaml
import random
import argparse
import os

def is_circle_overlapping(center_x, center_y, radius, existing_circles, min_distance=0):
    for x, y, r in existing_circles:
        distance = np.sqrt((center_x - x)**2 + (center_y - y)**2)
        if distance < (radius + r + min_distance):
            return True
    return False

def is_circle_in_bounds(center_x, center_y, radius, map_size_pixels, margin=0):
    return (center_x - radius - margin >= 0 and 
            center_x + radius + margin < map_size_pixels and 
            center_y - radius + margin >= 0 and 
            center_y + radius + margin < map_size_pixels)

def is_in_forbidden_zones(center_x, center_y, radius, forbidden_zones, cell_resolution_m, map_size_pixels):
    if not forbidden_zones:
        return False
    
    map_center_pixels = map_size_pixels / 2
    center_x_meters = (center_x - map_center_pixels) * cell_resolution_m
    center_y_meters = (map_size_pixels - center_y - map_center_pixels) * cell_resolution_m # Y-axis is inverted
    radius_meters = radius * cell_resolution_m
    
    for zone_x, zone_y, zone_radius in forbidden_zones:
        distance = np.sqrt((center_x_meters - zone_x)**2 + (center_y_meters - zone_y)**2)
        if distance < (radius_meters + zone_radius):
            return True
    return False

def generate_gazebo_world(output_world_path, circles_pixels, boundary_margin, 
                          map_size_meters, resolution, origin_x, origin_y, obstacle_height=1.0):
    map_height_pixels = int(map_size_meters / resolution)
    sdf_models = ""
    model_index = 0

    # 1. 添加边界墙模型
    if boundary_margin > 0:
        wall_thickness = boundary_margin
        inner_size = map_size_meters - 2 * wall_thickness
        
        # Top wall
        sdf_models += f"""
    <model name='wall_north'>
      <static>true</static>
      <pose>0 {map_size_meters/2 - wall_thickness/2} {obstacle_height/2} 0 0 0</pose>
      <link name='link'><collision name='collision'><geometry><box><size>{map_size_meters} {wall_thickness} {obstacle_height}</size></box></geometry></collision><visual name='visual'><geometry><box><size>{map_size_meters} {wall_thickness} {obstacle_height}</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual></link>
    </model>"""
        # Bottom wall
        sdf_models += f"""
    <model name='wall_south'>
      <static>true</static>
      <pose>0 {-map_size_meters/2 + wall_thickness/2} {obstacle_height/2} 0 0 0</pose>
      <link name='link'><collision name='collision'><geometry><box><size>{map_size_meters} {wall_thickness} {obstacle_height}</size></box></geometry></collision><visual name='visual'><geometry><box><size>{map_size_meters} {wall_thickness} {obstacle_height}</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual></link>
    </model>"""
        # East wall
        sdf_models += f"""
    <model name='wall_east'>
      <static>true</static>
      <pose>{map_size_meters/2 - wall_thickness/2} 0 {obstacle_height/2} 0 0 0</pose>
      <link name='link'><collision name='collision'><geometry><box><size>{wall_thickness} {inner_size} {obstacle_height}</size></box></geometry></collision><visual name='visual'><geometry><box><size>{wall_thickness} {inner_size} {obstacle_height}</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual></link>
    </model>"""
        # West wall
        sdf_models += f"""
    <model name='wall_west'>
      <static>true</static>
      <pose>{-map_size_meters/2 + wall_thickness/2} 0 {obstacle_height/2} 0 0 0</pose>
      <link name='link'><collision name='collision'><geometry><box><size>{wall_thickness} {inner_size} {obstacle_height}</size></box></geometry></collision><visual name='visual'><geometry><box><size>{wall_thickness} {inner_size} {obstacle_height}</size></box></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual></link>
    </model>"""
        model_index = 4

    # 2. 添加圆形障碍物模型
    for c_px, c_py, r_px in circles_pixels:
        radius_m = r_px * resolution
        # 转换像素坐标到Gazebo世界坐标
        center_x_m = origin_x + c_px * resolution
        center_y_m = origin_y + (map_height_pixels - c_py) * resolution

        sdf_models += f"""
    <model name='obstacle_{model_index}'>
      <static>true</static>
      <pose>{center_x_m} {center_y_m} {obstacle_height / 2.0} 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><cylinder><radius>{radius_m}</radius><length>{obstacle_height}</length></cylinder></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
        <collision name='collision'>
          <geometry><cylinder><radius>{radius_m}</radius><length>{obstacle_height}</length></cylinder></geometry>
        </collision>
      </link>
    </model>"""
        model_index += 1

    # 3. 组装并写入 .world 文件
    world_template = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    {sdf_models}
  </world>
</sdf>
"""
    output_dir = os.path.dirname(output_world_path)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    with open(output_world_path, 'w') as f:
        f.write(world_template)
    
    print(f"Gazebo world file successfully generated at: {output_world_path}")

def generate_map_and_world(map_size_meters, cell_resolution_m, num_obstacles, 
                           obstacle_radius_min, obstacle_radius_max, 
                           robot_radius_m, output_filename, world_output_filename,
                           boundary_margin=0.0, forbidden_zones=None, obstacle_height=1.0):
    
    # --- 1. 生成 PGM 地图 ---
    output_dir = os.path.dirname(output_filename)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    base_filename = os.path.basename(output_filename)

    map_size_pixels = int(map_size_meters / cell_resolution_m)
    map_image = np.ones((map_size_pixels, map_size_pixels), dtype=np.uint8) * 255
    
    boundary_pixels = int(boundary_margin / cell_resolution_m)
    if boundary_margin > 0:
        map_image[0:boundary_pixels, :] = 0
        map_image[-boundary_pixels:, :] = 0
        map_image[:, 0:boundary_pixels] = 0
        map_image[:, -boundary_pixels:] = 0
        print(f"已添加边界障碍物，宽度为 {boundary_margin} 米 ({boundary_pixels} 像素)")
    
    existing_circles = []
    max_attempts = 1000
    min_distance_pixels = int(robot_radius_m / cell_resolution_m)
    
    for _ in range(num_obstacles):
        attempts = 0
        while attempts < max_attempts:
            # 确保障碍物在边界内部生成
            margin_pixels = boundary_pixels + 5 # 额外留出一点空隙
            radius_pixels = random.randint(int(obstacle_radius_min / cell_resolution_m), 
                                           int(obstacle_radius_max / cell_resolution_m))
            center_x = random.randint(margin_pixels + radius_pixels, map_size_pixels - margin_pixels - radius_pixels - 1)
            center_y = random.randint(margin_pixels + radius_pixels, map_size_pixels - margin_pixels - radius_pixels - 1)
            
            if (not is_circle_overlapping(center_x, center_y, radius_pixels, existing_circles, min_distance_pixels) and
                not is_in_forbidden_zones(center_x, center_y, radius_pixels, forbidden_zones, 
                                          cell_resolution_m, map_size_pixels)):
                cv2.circle(map_image, (center_x, center_y), radius_pixels, 0, -1)
                existing_circles.append((center_x, center_y, radius_pixels))
                break
            attempts += 1
        
        if attempts >= max_attempts:
            print(f"警告：无法生成第 {len(existing_circles) + 1} 个不重叠的障碍物")
            break
    
    print(f"成功生成 {len(existing_circles)} 个圆形障碍物")
    
    cv2.imwrite(output_filename + ".pgm", map_image)
    print(f"PGM map generated at: {output_filename}.pgm")

    # --- 2. 创建 YAML 配置文件 ---
    origin_x = -map_size_meters / 2
    origin_y = -map_size_meters / 2
    
    with open(output_filename + ".yaml", 'w') as f:
        f.write(f"image: {base_filename}.pgm\n")
        f.write(f"resolution: {cell_resolution_m:.6f}\n")
        f.write(f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
    print(f"YAML config generated at: {output_filename}.yaml")

    # --- 3. 生成 Gazebo World 文件 ---
    generate_gazebo_world(world_output_filename, existing_circles, boundary_margin,
                          map_size_meters, cell_resolution_m, origin_x, origin_y, obstacle_height)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='生成圆形障碍物地图并直接转换为Gazebo world文件')
    parser.add_argument('--size', type=float, default=22.0, help='地图边长（米）')
    parser.add_argument('--resolution', type=float, default=0.05, help='栅格分辨率（米/像素）')
    parser.add_argument('--obstacles', type=int, default=10, help='障碍物数量')
    parser.add_argument('--min_radius', type=float, default=1.0, help='障碍物最小半径（米）')
    parser.add_argument('--max_radius', type=float, default=2.0, help='障碍物最大半径（米）')
    parser.add_argument('--robot_radius', type=float, default=0.18, help='用于计算障碍物最小间距的机器人半径（米）')
    parser.add_argument('--output', type=str, required=True, help='输出PGM/YAML文件的基本路径和名称 (例如: ./maps/my_map)')
    parser.add_argument('--world_output', type=str, help='输出.world文件的完整路径和名称 (可选, 默认基于--output生成)')
    parser.add_argument('--boundary', type=float, default=1.0, help='边界障碍物宽度（米），设为0表示无边界')
    parser.add_argument('--height', type=float, default=1.0, help='Gazebo中障碍物的高度（米）')
    
    args = parser.parse_args()
    
    world_output_path = args.world_output
    if not world_output_path:
        world_output_path = os.path.splitext(args.output)[0] + ".world"

    # 示例：定义禁止区域（可根据需要修改）
    # 格式：[(x坐标(米), y坐标(米), 半径(米)), ...]，坐标相对于地图中心
    forbidden_zones = [
        (-9, -9, 1.0),
        (9, 9, 1.0)
    ]
    
    generate_map_and_world(
        map_size_meters=args.size,
        cell_resolution_m=args.resolution,
        num_obstacles=args.obstacles,
        obstacle_radius_min=args.min_radius,
        obstacle_radius_max=args.max_radius,
        robot_radius_m=args.robot_radius,
        output_filename=args.output,
        world_output_filename=world_output_path,
        boundary_margin=args.boundary,
        forbidden_zones=forbidden_zones,
        obstacle_height=args.height
    )
