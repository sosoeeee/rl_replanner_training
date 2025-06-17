import numpy as np
import cv2
import yaml
import random
import argparse
from PIL import Image
import os

def is_circle_overlapping(center_x, center_y, radius, existing_circles, min_distance=0):
    """
    检查新生成的圆是否与现有圆重叠
    
    参数:
        center_x, center_y: 新圆的圆心坐标
        radius: 新圆的半径
        existing_circles: 现有圆的列表，每个元素为 (x, y, r)
        min_distance: 圆之间的最小距离
    
    返回:
        bool: 如果重叠返回True，否则返回False
    """
    for x, y, r in existing_circles:
        # 计算圆心距
        distance = np.sqrt((center_x - x)**2 + (center_y - y)**2)
        # 判断重叠
        if distance < (radius + r + min_distance):
            return True
    return False

def is_circle_in_bounds(center_x, center_y, radius, map_size_pixels, margin=0):
    """
    检查圆是否在地图边界内
    
    参数:
        center_x, center_y: 圆心坐标
        radius: 圆的半径
        map_size_pixels: 地图尺寸（像素）
        margin: 边界预留的安全距离
    
    返回:
        bool: 如果圆完全在地图内返回True，否则返回False
    """
    return (center_x - radius - margin >= 0 and 
            center_x + radius + margin < map_size_pixels and 
            center_y - radius - margin >= 0 and 
            center_y + radius + margin < map_size_pixels)

def generate_map(map_size_meters, cell_resolution_m, num_obstacles, 
                 obstacle_radius_min, obstacle_radius_max, 
                 robot_radius_m, inflation_radius_m, output_filename,
                 boundary_margin=0.0):
    """
    生成ROS可使用的地图并配置相应的参数
    
    参数:
        map_size_meters: 地图大小（米）
        cell_resolution_m: 每个栅格单元的分辨率（米/像素）
        num_obstacles: 生成的障碍物数量
        obstacle_radius_min: 最小障碍物半径（米）
        obstacle_radius_max: 最大障碍物半径（米）
        robot_radius_m: 机器人半径（米）
        inflation_radius_m: 膨胀半径（米）
        output_filename: 输出地图文件名
        boundary_margin: 边界障碍物距离地图边缘的距离（米）
    """
    # 保存路径
    save_dir = "./tb3_classic"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    output_path = os.path.join(save_dir, output_filename)

    map_size_pixels = int(map_size_meters / cell_resolution_m)
    map_image = np.ones((map_size_pixels, map_size_pixels), dtype=np.uint8) * 255
    
    # 添加边界障碍物
    if boundary_margin > 0:
        boundary_pixels = int(boundary_margin / cell_resolution_m)
        
        map_image[0:boundary_pixels, :] = 0
        map_image[-boundary_pixels:, :] = 0
        map_image[:, 0:boundary_pixels] = 0
        map_image[:, -boundary_pixels:] = 0
        
        print(f"已添加边界障碍物，宽度为 {boundary_margin} 米 ({boundary_pixels} 像素)")
    
    # 存储已生成的圆的信息
    existing_circles = []
    max_attempts = 1000
    min_distance = int(robot_radius_m / cell_resolution_m)
    
    # 生成随机圆形障碍物
    for _ in range(num_obstacles):
        attempts = 0
        while attempts < max_attempts:
            margin_pixels = int(boundary_margin / cell_resolution_m)
            center_x = random.randint(margin_pixels, map_size_pixels - margin_pixels - 1)
            center_y = random.randint(margin_pixels, map_size_pixels - margin_pixels - 1)
            radius = random.randint(int(obstacle_radius_min / cell_resolution_m), 
                                  int(obstacle_radius_max / cell_resolution_m))
            
            # 检查是否重叠
            if (is_circle_in_bounds(center_x, center_y, radius, map_size_pixels, margin_pixels) and 
                not is_circle_overlapping(center_x, center_y, radius, existing_circles, min_distance)):
                cv2.circle(map_image, (center_x, center_y), radius, 0, -1)
                existing_circles.append((center_x, center_y, radius))
                break
            
            attempts += 1
        
        if attempts >= max_attempts:
            print(f"警告：无法生成第 {len(existing_circles) + 1} 个不重叠的障碍物")
            break
    
    print(f"成功生成 {len(existing_circles)} 个障碍物")
    
    # 保存PGM文件
    cv2.imwrite(output_path + ".pgm", map_image)
    
    # 创建YAML配置文件
    origin_x = -map_size_meters / 2
    origin_y = -map_size_meters / 2
    
    with open(output_path + ".yaml", 'w') as f:
        f.write(f"image: {output_filename}.pgm\n")
        f.write(f"resolution: {cell_resolution_m:.6f}\n")
        f.write(f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]\n")
        f.write(f"negate: 0\n")
        f.write(f"occupied_thresh: 0.65\n")
        f.write(f"free_thresh: 0.196\n\n")
        
        # 写入static_layer
        f.write("static_layer:\n")
        f.write("  track_unknown_space: true\n")
        f.write("  use_maximum: false\n")
        f.write("  trinary_costmap: true\n")
        f.write("  unknown_cost_value: -1\n")
        f.write("  lethal_threshold: 100\n\n")
        
        # 写入inflation_layer
        f.write("inflation_layer:\n")
        f.write(f"  robot_radius: {robot_radius_m}\n")
        f.write(f"  inflation_radius: {inflation_radius_m}  # make sure this is larger than robot_radius\n")
        f.write("  cost_scaling_factor: 3.0\n")
        f.write("  inflate_unknown: false\n")
        f.write("  inflate_around_unknown: false\n")
    
    print(f"地图生成成功: {output_path}.pgm 及配套 YAML 配置文件")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='生成ROS地图')
    parser.add_argument('--size', type=float, default=20.0, 
                       help='地图边长（米）')
    parser.add_argument('--resolution', type=float, default=0.05, 
                       help='栅格分辨率（米/像素）')
    parser.add_argument('--obstacles', type=int, default=5, 
                       help='障碍物数量')
    parser.add_argument('--min_radius', type=float, default=0.5, 
                       help='障碍物最小半径（米）')
    parser.add_argument('--max_radius', type=float, default=1.0, 
                       help='障碍物最大半径（米）')
    parser.add_argument('--robot_radius', type=float, default=0.18, 
                       help='机器人半径（米）')
    parser.add_argument('--inflation_radius', type=float, default=1, 
                       help='膨胀半径（米）')
    parser.add_argument('--output', type=str, default='turtlebot3_world_circle', 
                       help='输出文件名')
    parser.add_argument('--boundary', type=float, default=2.0,
                       help='边界障碍物距离地图边缘的距离（米），设为0表示无边界障碍物')
    
    args = parser.parse_args()
    
    # 生成地图
    generate_map(
        map_size_meters=args.size,
        cell_resolution_m=args.resolution,
        num_obstacles=args.obstacles,
        obstacle_radius_min=args.min_radius,
        obstacle_radius_max=args.max_radius,
        robot_radius_m=args.robot_radius,
        inflation_radius_m=args.inflation_radius,
        output_filename=args.output,
        boundary_margin=args.boundary
    )