import glob
import time
import numpy as np
from typing import Dict, Union
import math
import copy
import gymnasium as gym
from gymnasium import spaces

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
        ):
        # addtional parameters
        self.eval_path_directory = eval_path_directory

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
    
    def _init_human_traj(self):
        # human path
        map_name = self.map_setting_file.split('/')[-1].split('.')[0]
        self.eval_path_directory = map_name + '/' + self.eval_path_directory
        if self.use_generator:
            self.replay_traj_files = glob.glob(self.replay_traj_path + '/' + self.eval_path_directory + '/*.txt')
        else:
            self.replay_traj_files = glob.glob(self.replay_traj_path + '/*.txt')

        self.traj_index = 0

    def _reset_human_traj(self, seed=None, options=None):
        if self.render_mode == "ros":
            # evaluate in order
            self.traj_index = (self.traj_index + 1) % len(self.replay_traj_files)
            print("\n\n ======================== Resetting trajectory: {} ======================== \n\n".format(self.traj_index))
        else:
            # Reset the trajectory index
            self.traj_index = np.random.randint(0, len(self.replay_traj_files))

        traj_file = self.replay_traj_files[self.traj_index]
        self.current_human_traj = np.loadtxt(traj_file)

        self.replan_num = 0
        self.current_step = 0
        self.total_reward_before_normalization = 0.0

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
            }
        else:
            self.info = {}

    def _calculate_reward(self, end_reward, is_terminal=False):
        # task reward
        if not is_terminal:
            eval_length = min(self.robot_prediction_length, len(self.current_robot_path) - self.robot_closest_idx)
            h_p = self._get_future_human_path(eval_length)
            r_p = self.future_robot_path_buffer[:eval_length]
            exp_error = np.exp(- self.exp_factor * np.linalg.norm((np.array(h_p).reshape((-1,2)) - np.array(r_p).reshape((-1,2))), axis=1))
            decay_weight = [self.decay_factor ** i for i in range(eval_length)] 
            decay_weight = np.array(decay_weight) * (1 - self.decay_factor) / (1 - self.decay_factor ** (eval_length))
            task_reward = decay_weight.dot(exp_error) * self.reward_weight['task']
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

            





