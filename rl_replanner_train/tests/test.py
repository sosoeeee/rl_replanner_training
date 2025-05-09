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

def visualize_voronoi_graph(costmap, voronoi_nodes, title, save_path=None):
    """可视化Voronoi图"""
    plt.figure(figsize=(10, 10))
    
    # 绘制costmap
    costmap_data = np.zeros((costmap.getSizeInCellsY(), costmap.getSizeInCellsX()))
    for y in range(costmap.getSizeInCellsY()):
        for x in range(costmap.getSizeInCellsX()):
            costmap_data[y, x] = costmap.getCost(x, y)
    
    plt.imshow(costmap_data, cmap='gray', origin='lower')
    
    # 绘制Voronoi节点和连接
    for node in voronoi_nodes:
        # 绘制节点
        pos = node.getPosition()
        plt.plot(pos.x, pos.y, 'ro', markersize=5)
        
        # 绘制连接
        for adj in node.getAllAdjacent():
            adj_node = next(n for n in voronoi_nodes if n.getId() == adj[0])
            adj_pos = adj_node.getPosition()
            plt.plot([pos.x, adj_pos.x], [pos.y, adj_pos.y], 'b-', linewidth=1)
    
    plt.title(title)
    if save_path:
        plt.savefig(save_path)
    plt.show()

def main():
    rclpy.init()
    render_node = rclpy.create_node("test_node")
    
    # 初始化轨迹生成器
    traj_generator = cpp_utils.TrajGenerator()
    traj_generator.initialize(
        map_file="/home/rosdev/ros2_ws/rl_replanner_train/maps/tb3_classic/turtlebot3_world.yaml",
        planner_file="/home/rosdev/ros2_ws/cpp_utils/include/teb_local_planner/teb_params.yaml",
        path_resolution=0.025,
        time_resolution=0.1,
    )
    
    # 设置起点和终点
    startPoint = cpp_utils.Point(-1.72, -0.217)
    endPoint = cpp_utils.Point(1.96, 0.395)
    
    # 获取原始Voronoi图并可视化
    original_voronoi = traj_generator.get_voronoi_graph()
    original_voronoi.visualizeVoronoi("original_voronoi.ppm")
    print("Original Voronoi graph saved to original_voronoi.ppm")
    
    # 直接生成修改后的Voronoi图
    traj_generator.generate_modified_voronoi_graph(startPoint, endPoint)
    
    # 获取修改后的Voronoi图
    modified_voronoi = traj_generator.get_voronoi_graph()
    modified_voronoi.visualizeVoronoiModified("modified_voronoi.ppm")
    print("Modified Voronoi graph saved to modified_voronoi.ppm")
    
    # 获取起点和终点特有的节点
    start_neighbors = modified_voronoi.getStartNeighborNodes()
    end_neighbors = modified_voronoi.getEndNeighborNodes()
    
    print("\nAnalysis of Voronoi nodes:")
    print(f"Original graph has {len(original_voronoi.getAllNodes())} nodes")
    print(f"Modified graph has {len(modified_voronoi.getAllNodes())} nodes")
    print(f"Start point has {len(start_neighbors)} unique neighbors")
    print(f"End point has {len(end_neighbors)} unique neighbors")
    
    # 打印起点特有的节点信息
    print("\nStart point unique neighbors:")
    for node in start_neighbors:
        print(f"Node {node.getId()} at ({node.getPosition().x}, {node.getPosition().y})")
    
    # 打印终点特有的节点信息
    print("\nEnd point unique neighbors:")
    for node in end_neighbors:
        print(f"Node {node.getId()} at ({node.getPosition().x}, {node.getPosition().y})")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
