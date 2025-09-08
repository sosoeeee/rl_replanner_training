#!/usr/bin/env python3

import math
from lxml import etree

# 参数配置
BASE_WORLD_FILE = '/home/rosdev/ros2_ws/train_env/rl_replanner_training/rl_replanner_train/maps/sim_maps/circle_clutter.world'
MARKED_PATH_FILE = '/home/rosdev/ros2_ws/train_env/rl_replanner_training/rl_replanner_train/gennerate_marked_paths/marked_path.txt'
OUTPUT_WORLD_FILE = '/home/rosdev/ros2_ws/train_env/rl_replanner_training/rl_replanner_train/maps/sim_maps/circle_clutter_with_path.world'
SAMPLING_RATE = 5
SCALE_FACTOR = 0.5

# 标记点
YELLOW_DOT_LOCATIONS = [
    (-4.0, -4.0), (-1.6, 0.0), (-3.0, 1.4), (-4.0, -0.4),
    (3.4, -3.8), (4.0, -2.0), (-0.4, 1.6), (4.0, 4.0),
]

# 模型
def get_painted_floor_segment_sdf(index, point1, point2):
    """生成蓝色路径段的SDF字符串"""
    entity_name = f"painted_floor_segment_{index}"
    base_length = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2) * 2.2
    angle = math.atan2(point2[1] - point1[1], point2[0] - point1[0])
    center_x = (point1[0] + point2[0])
    center_y = (point1[1] + point2[1])
    center_z = 0.0001
    
    return f"""
    <model name='{entity_name}'>
      <static>true</static>
      <pose>{center_x} {center_y} {center_z} 0 0 {angle}</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{base_length} 0.1 0.0001</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0.5 1 1.0</ambient>
            <diffuse>0 0.5 1 1.0</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """

def get_yellow_dot_sdf(index, point):
    """生成黄色圆点标记的SDF字符串"""
    entity_name = f"yellow_dot_marker_{index}"
    center_x, center_y, center_z = point[0], point[1], 0.00015
    
    return f"""
    <model name='{entity_name}'>
      <static>true</static>
      <pose>{center_x} {center_y} {center_z} 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>0.08</radius>
              <length>0.0001</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient>
            <diffuse>1 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    """


def main():
    print(f"1. 正在读取基础世界文件: {BASE_WORLD_FILE}")
    try:
        parser = etree.XMLParser(remove_blank_text=True)
        tree = etree.parse(BASE_WORLD_FILE, parser)
        world_root = tree.find('world')
        if world_root is None:
            print("错误: 在基础世界文件中未找到 <world> 标签。")
            return
    except Exception as e:
        print(f"错误: 读取或解析基础世界文件失败: {e}")
        return

    models_to_add = []
    model_index = 0

    print(f"2. 正在处理路径文件以生成蓝色路径: {MARKED_PATH_FILE}")
    try:
        with open(MARKED_PATH_FILE, 'r') as f:
            lines = f.readlines()
        
        points = []
        for i, line in enumerate(lines):
            if i == 0 or (i + 1) == len(lines) or i % SAMPLING_RATE == 0:
                parts = line.strip().split()
                if len(parts) >= 2:
                    x = float(parts[0]) * SCALE_FACTOR
                    y = float(parts[1]) * SCALE_FACTOR
                    points.append((x, y))

        print(f"   - 已采样 {len(points)} 个路径点用于生成蓝色路径。")

        print(f"   - 正在添加 {len(YELLOW_DOT_LOCATIONS)} 个固定的黄色标记点。")
        for point in YELLOW_DOT_LOCATIONS:
            sdf_string = get_yellow_dot_sdf(model_index, point)
            models_to_add.append(sdf_string)
            model_index += 1

        for i in range(len(points) - 1):
            p1 = (points[i][0], points[i][1], 0.1)
            p2 = (points[i+1][0], points[i+1][1], 0.1)
            sdf_string = get_painted_floor_segment_sdf(model_index, p1, p2)
            models_to_add.append(sdf_string)
            model_index += 1

    except FileNotFoundError:
        print(f"警告: 未找到路径文件 {MARKED_PATH_FILE}，将跳过路径生成。")
    except Exception as e:
        print(f"错误: 处理路径文件时发生错误: {e}")

    print(f"4. 正在将 {len(models_to_add)} 个新模型添加到世界中...")
    for model_sdf in models_to_add:
        try:
            model_node = etree.fromstring(model_sdf)
            world_root.append(model_node)
        except etree.XMLSyntaxError as e:
            print(f"警告: 无效的SDF片段，已跳过: {e}")
            print(f"问题片段: {model_sdf}")

    print(f"5. 正在将最终的世界保存到: {OUTPUT_WORLD_FILE}")
    try:
        tree.write(OUTPUT_WORLD_FILE, pretty_print=True, xml_declaration=True, encoding='UTF-8')
        print("成功！新的世界文件已生成。")
    except Exception as e:
        print(f"错误: 写入输出文件失败: {e}")

if __name__ == '__main__':
    main()
