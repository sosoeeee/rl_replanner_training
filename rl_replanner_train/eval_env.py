import glob
import time
import numpy as np
from typing import Dict, Union
import math
import copy
import gymnasium as gym
from gymnasium import spaces
import yaml
from PIL import Image
from pathlib import Path

from rl_replanner_train.action_converter import ActionConverter
import cpp_utils

# render modules    
import psutil
import rclpy
from rl_replanner_train.render.rosRender import rosRender, generate_rviz_launch_description
from launch import LaunchService
from rl_replanner_train.base_env import BaseEnv


# Action Id
DO_NOTHING = 0
LOCAL_GOAL = 1

# simulation world for evaluation
# TODO: To increase the efficiency of the evaluation, generate the human trajectory in advance and save it to a file.
# TODO: change the terminal condition. Robot need interact with all possible human trajectories of different homotopies, once for each trajectory.
# TODO: When evlauating, the gamma is set to 1.0. (See Stable-Baselines3: evaluate_policy() function for more details)

# TODO: Considerating the evaluation speed, robot don't need to interact with all possible human trajectories of different homotopies. 
# Instead, a random trajectory is selected from the replay buffer and the robot interacts with it. 
class EvalEnv(BaseEnv):
    def __init__(
            self, 
            reward_weight, 
            map_setting_file,
            path_planner_setting_file,
            eval_path_directory='eval_paths', # only used when evaluating
            render_mode=None, 
            obser_width=5, 
            map_resolution=0.05, 
            human_history_length=20,
            robot_prediction_length=150,
            speed_buffer_length=4,
            replay_traj_path=None,
            decision_interval=1,
            render_real_time_factor=1.0,
            use_generator = False,
            eval_ordered=False,  # if True, the evaluation will be in order of the eval_path_directory
            visualize_cones=False,
            visualize_heatmap=True, # if True, replan heatmap will be generated
            start_traj_idx=0,
        ):
        # addtional parameters
        self.eval_path_directory = eval_path_directory
        self.eval_ordered = eval_ordered
        self.prediction_errors = []
        self.visualize_cones = visualize_cones
        self.visualize_heatmap = visualize_heatmap
        self.start_traj_idx = start_traj_idx
        self.replan_angles = [] # Initialize list to store replan angles
        if self.visualize_cones:
            self.cone_history = []

        super().__init__(
            reward_weight=reward_weight,
            map_setting_file=map_setting_file,
            path_planner_setting_file=path_planner_setting_file,
            obser_width=obser_width,
            map_resolution=map_resolution,
            human_history_length=human_history_length,
            robot_prediction_length=robot_prediction_length,
            speed_buffer_length=speed_buffer_length,
            replay_traj_path=replay_traj_path,
            decision_interval=decision_interval,
            use_generator = use_generator,
            render_mode=render_mode,
            render_real_time_factor=render_real_time_factor,
        )
        
        self.replan_heatmap = None
        self.map_meta_data = None

        if self.visualize_heatmap:
            # Initialize replan heatmap and store map metadata by reading the map file
            try:
                with open(map_setting_file, 'r') as f:
                    map_config = yaml.safe_load(f)
                
                map_image_path_str = map_config['image']
                
                map_setting_path = Path(map_setting_file)
                if not Path(map_image_path_str).is_absolute():
                    map_image_path = map_setting_path.parent / map_image_path_str
                else:
                    map_image_path = Path(map_image_path_str)

                with Image.open(map_image_path) as img:
                    width, height = img.size

                self.replan_heatmap = np.zeros((height, width), dtype=np.int32)
                
                # Store metadata needed for coordinate conversion
                self.map_meta_data = {
                    'resolution': map_config['resolution'],
                    'origin': map_config['origin'], # [x, y, yaw]
                    'height': height,
                    'width': width
                }
            except Exception as e:
                raise RuntimeError(f"Failed to load map for heatmap initialization: {e}")

    def _world_to_map(self, world_x, world_y):
        """Converts world coordinates to map pixel coordinates."""
        meta = self.map_meta_data
        origin_x = meta['origin'][0]
        origin_y = meta['origin'][1]
        resolution = meta['resolution']
        height = meta['height']

        map_x = int((world_x - origin_x) / resolution)
        map_y = int(height - (world_y - origin_y) / resolution)
        
        return map_x, map_y

    def _init_human_traj(self):
        # human path
        map_name = self.map_setting_file.split('/')[-1].split('.')[0]
        self.eval_path_directory = map_name + '/' + self.eval_path_directory
        if self.use_generator:
            self.replay_traj_files = glob.glob(self.replay_traj_path + '/' + self.eval_path_directory + '/*.txt')
        else:
            self.replay_traj_files = glob.glob(self.replay_traj_path + '/' + map_name + '/collected_paths/*.txt')

        self.traj_index = self.start_traj_idx -1

    def _reset_human_traj(self, seed=None, options=None):
        if self.render_mode == "ros":
            self.eval_ordered = True  # when rendering in ROS, always evaluate in order
            print("Warning: eval_ordered is set to True when rendering in ROS.")

        if self.eval_ordered:
            # evaluate in order
            self.traj_index = (self.traj_index + 1) % len(self.replay_traj_files)
            print("\n\n ======================== Resetting trajectory: {} ======================== \n\n".format(self.traj_index))
        else:
            # Reset the trajectory index
            self.traj_index = np.random.randint(0, len(self.replay_traj_files))

        traj_file = self.replay_traj_files[self.traj_index]
        self.current_human_traj = np.loadtxt(traj_file)

        self.replan_num = 0
        self.fail_num = 0
        self.current_step = 0
        self.total_reward_before_normalization = 0.0
        self.prediction_errors = []
        self.replan_angles = [] # Reset for each new trajectory
        # Reset heatmap for the new trajectory
        if hasattr(self, 'replan_heatmap'):
            self.replan_heatmap.fill(0)
        if self.visualize_cones:
            self.cone_history = []

    # when evaluating, if the robot action is invalid, current episode will be terminated
    def _interact(self):
        # This function is used to interact with the environment
        self.cur_position = [self.human_path_buffer[-1][0], self.human_path_buffer[-1][1]] 

        terminated = False

        # apply action
        # direction vector is average velocity calculated from past trajectory
        self._get_robot_direction()

        if self.current_action[0] == LOCAL_GOAL:
            # Record replan event position
            if self.visualize_heatmap and self.replan_heatmap is not None:
                map_x, map_y = self._world_to_map(self.cur_position[0], self.cur_position[1])
                if 0 <= map_y < self.replan_heatmap.shape[0] and 0 <= map_x < self.replan_heatmap.shape[1]:
                    self.replan_heatmap[map_y, map_x] += 1

            # rescale to the map size
            self.current_action[1][0] = self.current_action[1][0] * self.obser_width
            self.current_action[1][1] = self.current_action[1][1] * self.obser_width

            if self._get_predicted_goal(depth=self.current_action[1][0], radius=self.current_action[1][1]):
                # Calculate and store the replan angle
                p1 = np.array(self.cur_position)
                center = np.array(self.cone_center)
                radius = self.current_action[1][1]
                dist_to_center = np.linalg.norm(center - p1)

                angle = 2 * np.arctan(radius / dist_to_center)
                self.replan_angles.append(np.rad2deg(angle)) # Store angle in degrees

                if self.visualize_cones:
                    # Calculate triangle vertices for visualization
                    vec_to_center = center - p1
                    dist_to_center = np.linalg.norm(vec_to_center)

                    if dist_to_center > radius:
                        angle_p1_center = np.arctan2(vec_to_center[1], vec_to_center[0])
                        angle_offset = np.arcsin(radius / dist_to_center)
                        
                        angle1 = angle_p1_center - angle_offset
                        angle2 = angle_p1_center + angle_offset

                        tangent_len = np.sqrt(dist_to_center**2 - radius**2)

                        p2 = p1 + tangent_len * np.array([np.cos(angle1), np.sin(angle1)])
                        p3 = p1 + tangent_len * np.array([np.cos(angle2), np.sin(angle2)])
                        
                        self.cone_history.append([p1.tolist(), p2.tolist(), p3.tolist()])

                self.path_planner.loadCone(cone_center=self.cone_center, 
                                            current_pos=self.cur_position,
                                            radius=self.current_action[1][1],
                                            is_enabled=True)
                if not self._plan_robot_path([self.cur_position[0], self.cur_position[1]], self.pred_goal):
                    # terminated = True
                    self.fail_num += 1
                # use point on robot path as the start point
                # self._plan_robot_path([self.current_robot_path[self.robot_closest_idx][0], self.current_robot_path[self.robot_closest_idx][1]], self.pred_goal)
            else:
                # terminated = True
                self.fail_num += 1

        if terminated:
            end_reward = -1
        elif self._get_human_path():
            terminated = True
            end_reward = 1
        else:
            self.time += self.decision_interval
            end_reward = 0

        return terminated, end_reward
    
    def _get_info(self, end_reward=0, is_terminal=False):
        self.current_step += 1
        if self.current_action[0] == LOCAL_GOAL:
            self.replan_num += 1

        if is_terminal:
            if end_reward > 0:
                is_success = True
            else:
                is_success = False

            self.info = {
                'is_success': is_success,
                'replan_freq': self.replan_num / self.current_step,  # replan frequency
                'fail_rate': self.fail_num / self.current_step,  # fail rate
                'cur_idx': self.traj_index,
                'eval_traj_num': len(self.replay_traj_files),
                'robot_path_history': copy.deepcopy(self.human_path_buffer),
                'reference_traj': self.current_human_traj,
                'replan_angles': self.replan_angles, # Add replan angles to info
            }
            if self.visualize_heatmap:
                self.info['replan_heatmap'] = self.replan_heatmap.copy()
                self.info['map_setting_file'] = self.map_setting_file

            if self.visualize_cones:
                self.info['cone_history'] = self.cone_history
            avg_prediction_error = np.mean(self.prediction_errors) if self.prediction_errors else 0.0
            self.info['prediction_error'] = avg_prediction_error
        else:
            self.info = {}

    def _calculate_reward(self, end_reward, is_terminal=False):
        # task reward
        if not is_terminal:
            eval_length = min(self.robot_prediction_length, len(self.current_robot_path) - self.robot_closest_idx)
            h_p = self._get_future_human_path(eval_length)
            r_p = self.future_robot_path_buffer[:eval_length]

            # Calculate prediction error (mean distance)
            if len(h_p) > 0 and len(r_p) > 0:
                distance_error_per_step = np.linalg.norm((np.array(h_p).reshape((-1,2)) - np.array(r_p).reshape((-1,2))), axis=1)
                mean_distance_error = np.mean(distance_error_per_step)
                self.prediction_errors.append(mean_distance_error)
                exp_error = np.exp(- self.exp_factor * distance_error_per_step)
            else:
                distance_error_per_step = []
                exp_error = []
                
            if len(exp_error) > 0:
                decay_weight = [self.decay_factor ** i for i in range(eval_length)] 
                decay_weight = np.array(decay_weight) * (1 - self.decay_factor) / (1 - self.decay_factor ** (eval_length))
                task_reward = decay_weight.dot(exp_error)
            else:
                task_reward = 0.0
        else:
            task_reward = 0.0

        self.total_reward_before_normalization += task_reward

        # debug
        if self.render_mode == "ros":
            print("task_reward: ", task_reward)

        if not is_terminal:
            self.reward = 0.0
        else:
            self.reward = self.total_reward_before_normalization / (self.current_step + 1)







