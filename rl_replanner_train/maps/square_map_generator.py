'''
该文件用于生成矩形障碍物地图，使用示例：
python square_map_generator.py --size 30 --obstacles 15 --boundary 2.0 --output large_world

如需自定义禁止区域，请编辑代码中的 forbidden_zones 列表：
forbidden_zones = [(0, 0, 2.0), (-5, 3, 1.5)]  # 在指定位置禁止生成障碍物
'''

import numpy as np
import cv2
import yaml
import random
import argparse
from PIL import Image
import os

def is_rectangle_in_bounds(x, y, width, height, map_size_pixels, margin=0):
    """
    检查矩形是否在地图边界内
    
    参数:
        x, y: 矩形左上角坐标
        width, height: 矩形的宽度和高度
        map_size_pixels: 地图尺寸（像素）
        margin: 边界预留的安全距离
    
    返回:
        bool: 如果矩形完全在地图内返回True，否则返回False
    """
    return (x - margin >= 0 and 
            y - margin >= 0 and 
            x + width + margin < map_size_pixels and 
            y + height + margin < map_size_pixels)

def is_rectangle_overlapping(x, y, width, height, existing_rectangles, min_distance=0):
    """
    检查新生成的矩形是否与现有矩形重叠
    
    参数:
        x, y: 新矩形左上角坐标
        width, height: 新矩形的宽度和高度
        existing_rectangles: 现有矩形的列表，每个元素为 (x, y, w, h)
        min_distance: 矩形之间的最小距离
    
    返回:
        bool: 如果重叠返回True，否则返回False
    """
    for ex, ey, ew, eh in existing_rectangles:
        # 检查两个矩形是否重叠（考虑最小距离）
        if not (x + width + min_distance <= ex or 
                ex + ew + min_distance <= x or 
                y + height + min_distance <= ey or 
                ey + eh + min_distance <= y):
            return True
    return False

def is_rectangle_in_forbidden_zones(x, y, width, height, forbidden_zones, cell_resolution_m, map_size_pixels):
    """
    检查矩形障碍物是否与禁止区域重叠
    
    参数:
        x, y: 矩形左上角坐标（像素）
        width, height: 矩形的宽度和高度（像素）
        forbidden_zones: 禁止区域列表，每个元素为 (x_meters, y_meters, radius_meters)
        cell_resolution_m: 栅格分辨率（米/像素）
        map_size_pixels: 地图尺寸（像素）
    
    返回:
        bool: 如果与禁止区域重叠返回True，否则返回False
    
    使用示例:
        # 定义禁止区域：在地图中心半径2米、在(-5,3)位置半径1.5米
        forbidden_zones = [(0, 0, 2.0), (-5, 3, 1.5)]
    """
    if not forbidden_zones:
        return False
    
    # 将像素坐标转换为米坐标（相对于地图中心）
    map_center_pixels = map_size_pixels / 2
    
    # 矩形的四个角点（像素坐标）
    corners_pixels = [
        (x, y),                    # 左上角
        (x + width, y),            # 右上角
        (x, y + height),           # 左下角
        (x + width, y + height)    # 右下角
    ]
    
    # 矩形中心点
    center_x_pixels = x + width / 2
    center_y_pixels = y + height / 2
    
    for zone_x, zone_y, zone_radius in forbidden_zones:
        # 检查矩形中心是否在禁止区域内
        center_x_meters = (center_x_pixels - map_center_pixels) * cell_resolution_m
        center_y_meters = (center_y_pixels - map_center_pixels) * cell_resolution_m
        center_distance = np.sqrt((center_x_meters - zone_x)**2 + (center_y_meters - zone_y)**2)
        
        if center_distance < zone_radius:
            return True
        
        # 检查矩形的四个角点是否在禁止区域内
        for corner_x_pixels, corner_y_pixels in corners_pixels:
            corner_x_meters = (corner_x_pixels - map_center_pixels) * cell_resolution_m
            corner_y_meters = (corner_y_pixels - map_center_pixels) * cell_resolution_m
            corner_distance = np.sqrt((corner_x_meters - zone_x)**2 + (corner_y_meters - zone_y)**2)
            
            if corner_distance < zone_radius:
                return True
    
    return False

def generate_map(map_size_meters, cell_resolution_m, num_obstacles, 
                 obstacle_width_min, obstacle_width_max,
                 obstacle_height_min, obstacle_height_max,
                 robot_radius_m, inflation_radius_m, output_filename,
                 boundary_margin=0.0, forbidden_zones=None):
    """
    生成ROS可使用的地图并配置相应的参数
    
    参数:
        map_size_meters: 地图大小（米）
        cell_resolution_m: 每个栅格单元的分辨率（米/像素）
        num_obstacles: 生成的障碍物数量
        obstacle_width_min: 最小障碍物宽度（米）
        obstacle_width_max: 最大障碍物宽度（米）
        obstacle_height_min: 最小障碍物高度（米）
        obstacle_height_max: 最大障碍物高度（米）
        robot_radius_m: 机器人半径（米）
        inflation_radius_m: 膨胀半径（米）
        output_filename: 输出地图文件名
        boundary_margin: 边界障碍物距离地图边缘的距离（米）
        forbidden_zones: 禁止区域列表，每个元素为 (x_meters, y_meters, radius_meters)
    """
    # 保存路径
    save_dir = "./square_map" # 在maps路径下运行
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
    
    # 存储已生成的矩形的信息
    existing_rectangles = []
    max_attempts = 1000
    min_distance = int(robot_radius_m / cell_resolution_m)
    
    # 生成随机矩形障碍物
    for _ in range(num_obstacles):
        attempts = 0
        while attempts < max_attempts:
            margin_pixels = int(boundary_margin / cell_resolution_m)
            
            # 计算可用空间
            available_space = map_size_pixels - 2 * margin_pixels
            
            # 确保障碍物尺寸不超过可用空间
            max_width_pixels = min(int(obstacle_width_max / cell_resolution_m), available_space - 1)
            max_height_pixels = min(int(obstacle_height_max / cell_resolution_m), available_space - 1)
            min_width_pixels = max(int(obstacle_width_min / cell_resolution_m), 1)
            min_height_pixels = max(int(obstacle_height_min / cell_resolution_m), 1)
            
            # 检查是否有足够空间生成障碍物
            if max_width_pixels < min_width_pixels or max_height_pixels < min_height_pixels:
                print(f"警告：地图空间不足以生成指定尺寸的障碍物")
                break
            
            width = random.randint(min_width_pixels, max_width_pixels)
            height = random.randint(min_height_pixels, max_height_pixels)
            
            # 计算有效的位置范围
            max_x = map_size_pixels - width - margin_pixels - 1
            max_y = map_size_pixels - height - margin_pixels - 1
            
            if max_x < margin_pixels or max_y < margin_pixels:
                attempts += 1
                continue
                
            x = random.randint(margin_pixels, max_x)
            y = random.randint(margin_pixels, max_y)
            
            # 检查是否在边界内、不重叠且不在禁止区域内
            if (is_rectangle_in_bounds(x, y, width, height, map_size_pixels, margin_pixels) and
                not is_rectangle_overlapping(x, y, width, height, existing_rectangles, min_distance) and
                not is_rectangle_in_forbidden_zones(x, y, width, height, forbidden_zones, cell_resolution_m, map_size_pixels)):
                cv2.rectangle(map_image, (x, y), (x + width, y + height), 0, -1)
                existing_rectangles.append((x, y, width, height))
                break
            
            attempts += 1
        
        if attempts >= max_attempts:
            print(f"警告：无法生成第 {len(existing_rectangles) + 1} 个不重叠的障碍物")
            break
    
    print(f"成功生成 {len(existing_rectangles)} 个矩形障碍物")
    
    # 保存PGM
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
    parser.add_argument('--size', type=float, default=22.0, 
                       help='地图边长（米）')
    parser.add_argument('--resolution', type=float, default=0.05, 
                       help='栅格分辨率（米/像素）')
    parser.add_argument('--obstacles', type=int, default=10, 
                       help='障碍物数量')
    parser.add_argument('--min_width', type=float, default=1.0, 
                       help='障碍物最小宽度（米）')
    parser.add_argument('--max_width', type=float, default=4.0, 
                       help='障碍物最大宽度（米）')
    parser.add_argument('--min_height', type=float, default=1.0, 
                       help='障碍物最小高度（米）')
    parser.add_argument('--max_height', type=float, default=4.0, 
                       help='障碍物最大高度（米）')
    parser.add_argument('--robot_radius', type=float, default=0.18, 
                       help='机器人半径（米）')
    parser.add_argument('--inflation_radius', type=float, default=0.55, 
                       help='膨胀半径（米）')
    parser.add_argument('--output', type=str, default='turtlebot3_world_new', 
                       help='输出文件名')
    parser.add_argument('--boundary', type=float, default=1.0,
                       help='边界障碍物距离地图边缘的距离（米），设为0表示无边界障碍物')
    
    args = parser.parse_args()
    
    # 示例：定义禁止区域（可根据需要修改）
    # 格式：[(x坐标(米), y坐标(米), 半径(米)), ...]
    # 坐标相对于地图中心，例如：
    forbidden_zones = [
        (-9, -9, 1.0),
        (9, 9, 1.0)
        # (9, 9, 5.0),
        # (-9, -9, 5.0) 
    ]
    # 如果不需要禁止区域，可以设置为：forbidden_zones = None 或 forbidden_zones = []
    
    # 生成地图
    generate_map(
        map_size_meters=args.size,
        cell_resolution_m=args.resolution,
        num_obstacles=args.obstacles,
        obstacle_width_min=args.min_width,
        obstacle_width_max=args.max_width,
        obstacle_height_min=args.min_height,
        obstacle_height_max=args.max_height,
        robot_radius_m=args.robot_radius,
        inflation_radius_m=args.inflation_radius,
        output_filename=args.output,
        boundary_margin=args.boundary,
        forbidden_zones=forbidden_zones
    )
