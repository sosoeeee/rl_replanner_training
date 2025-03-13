import math
import threading

import cpp_utils
from rl_replanner_train.maps.costmap_2d import PyCostmap2D
# ros2 dependencies
import rclpy

# ros2 message
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

########################################################################################################
# rviz2 launch file
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

########################################################################################################


# A ros node responsible for rendering the simulation process
# imitate the structure of a ROS2 lifecycle node
class rosRender():
    def __init__(self):


        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('render_node')
    
    # global parameters
    def init_parameters(self):
        # set log level
        self.node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)

        self.node.declare_parameter('global_frame', 'map')
        self.global_frame_ = self.node.get_parameter('global_frame').get_parameter_value().string_value

    def on_configure(self, map: cpp_utils.Costmap2D_cpp):
        self.init_parameters()

        self.global_map_pub = self.node.create_publisher(OccupancyGrid, 'global_map', 10)
        self.global_map = PyCostmap2D(self.node)
        self.global_map.loadCostmapFromCostmapCpp(map)
        self.human_path_pub = self.node.create_publisher(Path, 'human_path', 10)
        self.robot_path_pub = self.node.create_publisher(Path, 'robot_path', 10)
        self.robot_pose_pub = self.node.create_publisher(PoseStamped, 'robot_pose', 10)
        self.robot_marker_pub = self.node.create_publisher(Marker, 'robot_marker', 10)
        self.marker = Marker()
        self.marker.header.frame_id = self.global_frame_
        self.marker.header.stamp = self.node.get_clock().now().to_msg()
        self.marker.ns = "robot"
        self.marker.id = 0
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.18  # Diameter of the circle
        self.marker.scale.y = 0.18  # Diameter of the circle
        self.marker.scale.z = 0.01  # Height of the cylinder (thin to represent a circle)
        self.marker.color.a = 0.3  # Alpha (transparency)
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        self.partial_map_pub = self.node.create_publisher(OccupancyGrid, 'obs_partial_map', 10)
        self.partial_map = PyCostmap2D(self.node)
        self.human_local_path_pub = self.node.create_publisher(Path, 'obs_human_path', 10)
        self.robot_local_path_pub = self.node.create_publisher(Path, 'obs_robot_path', 10)
        self.global_goal_pub = self.node.create_publisher(PoseStamped, 'obs_global_goal', 10)

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self._spin_node, args=(self.executor,), daemon=True)      

    def on_activate(self):
        self.spin_thread.start()
   
    def on_deactivate(self):
        self.executor.shutdown()
        self.spin_thread.join()

    def on_shutdown(self):
        self.node.destroy_node()

    def pub_global_map(self):
        self.global_map_pub.publish(self.global_map.getOccupancyGrid())
    
    def pub_global_map_with_cone(self, cur_pose: list[float], cone_center: list[float], cone_radius: float, inflated_distance: float):
        map_with_cone = self.global_map.load_cone_to_map(cur_pose[0], cur_pose[1], cone_center, cone_radius, inflated_distance)
        self.global_map_pub.publish(map_with_cone.getOccupancyGrid())

    def pub_partial_map(self, map: cpp_utils.Costmap2D_cpp):
        self.partial_map.loadCostmapFromCostmapCpp(map)
        self.partial_map_pub.publish(self.partial_map.getOccupancyGrid())

    def pub_human_path(self, human_traj: list[list[float]], path_resolution: float):
        path = Path()
        path.header.frame_id = self.global_frame_
        path.header.stamp = self.node.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.frame_id = self.global_frame_
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = human_traj[0][0]
        pose.pose.position.y = human_traj[0][1]
        path.poses.append(pose)

        for i in range(1, len(human_traj)):
            x = human_traj[i][0]
            y = human_traj[i][1]
            distance = ((x - path.poses[-1].pose.position.x)**2 + (y - path.poses[-1].pose.position.y)**2)**0.5
            if distance > path_resolution:
                pose = PoseStamped()
                pose.header.frame_id = self.global_frame_
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.pose.position.x = x
                pose.pose.position.y = y
                path.poses.append(pose)
        
        self.human_path_pub.publish(path)
    
    def pub_robot_path(self, robot_path: list[list[float]]):    
        self.robot_path_pub.publish(self._get_path_msg(robot_path))
    
    def pub_local_human_path(self, human_local_path: list[list[float]], robot_direrction: list[float]):
        self.human_local_path_pub.publish(self._get_path_msg(human_local_path))

        cur_pose = PoseStamped()
        cur_pose.header.frame_id = self.global_frame_
        cur_pose.header.stamp = self.node.get_clock().now().to_msg()
        cur_pose.pose.position.x = human_local_path[-1][0]
        cur_pose.pose.position.y = human_local_path[-1][1]
        cur_pose.pose.position.z = 0.0
        cur_pose.pose.orientation.x = 0.0
        cur_pose.pose.orientation.y = 0.0
        theta = math.atan2(robot_direrction[1], robot_direrction[0])
        cur_pose.pose.orientation.z = math.sin(theta / 2)
        cur_pose.pose.orientation.w = math.cos(theta / 2)

        self.robot_pose_pub.publish(cur_pose)

        self.marker.pose.position.x = cur_pose.pose.position.x
        self.marker.pose.position.y = cur_pose.pose.position.y
        self.robot_marker_pub.publish(self.marker)

    def pub_local_robot_path(self, robot_local_path: list[list[float]]):
        self.robot_local_path_pub.publish(self._get_path_msg(robot_local_path))

    def pub_global_goal(self, goal: list[float]):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame_
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        self.global_goal_pub.publish(goal_pose)

    # running in a sole thread
    def _spin_node(self, executor):
        try:
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            print('External shutdown requested.')
    
    def _get_path_msg(self, path: list[list[float]]):
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame_
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        for point in path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        return path_msg


def generate_rviz_launch_description():
    # Get the launch directory
    cur_dir = os.path.dirname(os.path.realpath(__file__))

    # Create the launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Declare the launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(cur_dir, 'default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    # Launch rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)

    # Add any conditioned actions
    ld.add_action(start_rviz_cmd)

    # Add other nodes and processes we need
    ld.add_action(exit_event_handler)

    return ld

    