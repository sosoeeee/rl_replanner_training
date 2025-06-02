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
from billiard import Process

from rl_replanner_train.base_env import BaseEnv


# Action Id
DO_NOTHING = 0
LOCAL_GOAL = 1


class TrainEnv(BaseEnv):
    def __init__(
            self, 
            reward_weight, 
            map_setting_file,
            path_planner_setting_file,
            traj_planner_setting_file, # only used when training
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
        self.traj_planner_setting_file = traj_planner_setting_file

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
        self.replay_traj_files = glob.glob(self.replay_traj_path + '/*.txt')

        # TODO: initialize the human traj generator
        if self.use_generator:
            self.traj_generator = cpp_utils.TrajGenerator()
            self.traj_generator.initialize(
                map_file=self.map_setting_file,
                planner_file=self.traj_planner_setting_file,
                path_resolution=self.path_resolution,
                time_resolution=self.time_resolution,
            )

    def _reset_human_traj(self, seed=None, options=None):
        # load human trajectory
        traj_file = np.random.choice(self.replay_traj_files)
        self.current_human_traj = np.loadtxt(traj_file)

        # TODO: generate a human trajectory. Its start and end point are the same as the trajectory loaded from the file
        if self.use_generator:
            start_point = cpp_utils.Point(self.current_human_traj[0][0], self.current_human_traj[0][1])
            end_point = cpp_utils.Point(self.current_human_traj[-1][0], self.current_human_traj[-1][1])
            generated_traj = self.traj_generator.sampleTraj(start=start_point, end=end_point)
            if generated_traj:
                traj_data = [[generated_traj[0].x, generated_traj[0].y, 0, 0, 0, 0, 0]]
                for i in range(1, len(generated_traj)):
                    x, y = generated_traj[i].x, generated_traj[i].y
                    last_x, last_y = generated_traj[i - 1].x, generated_traj[i - 1].y
                    dx = (x - last_x) / self.time_resolution
                    dy = (y - last_y) / self.time_resolution
                    theta = 0  
                    dtheta = 0  
                    t = i * self.time_resolution
                    traj_data.append([x, y, theta, dx, dy, dtheta, t])
                self.current_human_traj = np.array(traj_data)
            else:
                raise ValueError("[SimulationWorld] Failed to generate human trajectory.")

    def _get_info(self, end_reward=0, is_terminal=False):
        self.info = {}
    
    def _calculate_reward(self, end_reward, is_terminal=False):
        # task reward
        if not is_terminal:
            # exp_error = np.exp(- self.exp_factor * np.linalg.norm((self.structure_obs['human_path'].reshape((-1,2)) - self.structure_obs['robot_path'].reshape((-1,2))), axis=1))
            # # debug
            # # print("max error: ", np.max(np.linalg.norm((self.structure_obs['human_path'].reshape((-1,2)) - self.structure_obs['robot_path'].reshape((-1,2))), axis=1)))
            # # print("exp_error: ", exp_error)
            # # print("decay_weight: ", np.round(self.decay_weight, 2))
            # # print("decay_weight sum is: ", np.sum(self.decay_weight))
            # task_reward = self.decay_weight.dot(exp_error) * self.reward_weight['task']

            eval_length = min(self.robot_prediction_length, len(self.current_robot_path) - self.robot_closest_idx)
            h_p = self._get_future_human_path(eval_length)
            r_p = self.future_robot_path_buffer[:eval_length]
            exp_error = np.exp(- self.exp_factor * np.linalg.norm((np.array(h_p).reshape((-1,2)) - np.array(r_p).reshape((-1,2))), axis=1))
            decay_weight = [self.decay_factor ** i for i in range(eval_length)] 
            decay_weight = np.array(decay_weight) * (1 - self.decay_factor) / (1 - self.decay_factor ** (eval_length))
            task_reward = decay_weight.dot(exp_error) * self.reward_weight['task']

            # debug
            # if self.render_mode == 'ros':
            #     print("l2_error: ", np.linalg.norm((np.array(h_p).reshape((-1,2)) - np.array(r_p).reshape((-1,2))), axis=1))
                # print("exp_error: ", exp_error)
                # print("decay_weight: ", np.round(decay_weight, 2))
                # print("decay_weight sum is: ", np.sum(decay_weight))

        else:
            task_reward = 0.0

        # regularization reward
        # radius / depth
        if self.current_action[0] == LOCAL_GOAL:
            if "reg_angle" in self.reward_weight.keys():
                angle_reg_reward = -np.arctan(self.current_action[1][1] / self.current_action[1][0]) / (np.pi / 2) * self.reward_weight['reg_angle']
                # print("angle_reg_reward: ", angle_reg_reward)
            # change to the ln scale
            elif ("reg_angle_factor_a" in self.reward_weight.keys()) and ("reg_angle_factor_b" in self.reward_weight.keys()) and ("reg_angle_factor_k" in self.reward_weight.keys()):
                norm_angle = np.arctan(self.current_action[1][1] / self.current_action[1][0]) / (np.pi / 2)
                angle_reg_reward = self.reward_weight['reg_angle_factor_a'] + \
                                   self.angle_c_ * (np.log(1 - norm_angle)) / (1 - norm_angle) ** self.reward_weight['reg_angle_factor_b'] + \
                                   self.reward_weight['reg_angle_factor_k'] * np.log(norm_angle)
                
                angle_reg_reward = max(angle_reg_reward, -self.reward_weight['state'])

                if self.render_mode == "ros":
                    print("norm_angle: ", norm_angle)
            else:
                angle_reg_reward = 0.0
        else:
            angle_reg_reward = 0.0

        # depth
        # if self.current_action[0] == LOCAL_GOAL and self.current_action[1][0] < self.len_threshold:
        #     if "reg_depth" in self.reward_weight.keys():
        #         depth_reg_reward = (self.current_action[1][0] - self.len_threshold) / self.len_threshold * self.reward_weight['reg_depth']
        #     # change to the ln scale
        #     elif "reg_depth_factor" in self.reward_weight.keys():
        #         norm_depth = self.current_action[1][0] / self.len_threshold
        #         print("norm_depth: ", norm_depth)
        #         depth_reg_reward = 1 / self.reward_weight['reg_depth_factor'] * np.log(norm_depth)
        #     else:
        #         depth_reg_reward = 0.0
        # else:
        #     depth_reg_reward = 0.0

        # depth (smooth func)
        if self.current_action[0] == LOCAL_GOAL:
            if "reg_depth_factor_b" in self.reward_weight.keys():
                norm_depth = ((self.current_action[1][0] / self.obser_width) - 1e-3) / (np.sqrt(2)/2 - 1e-3)
                depth_reg_reward = self.depth_c_ * np.log(norm_depth) / (norm_depth) ** self.reward_weight['reg_depth_factor_b']
                if self.render_mode == "ros":
                    print("norm_depth: ", norm_depth)
            else:
                depth_reg_reward = 0.0
        else:
            depth_reg_reward = 0.0
        
        # # replan
        # if self.current_action[0] == DO_NOTHING:
        #     replan_reward = 0.0
        # else:
        #     replan_reward = -1 * self.reward_weight['reg_replan']
        
        self.reward = task_reward + angle_reg_reward + depth_reg_reward + end_reward * self.reward_weight['state']
        
        # debug
        # print("============== reward terms ==============")
        if self.render_mode == "ros":
            print("task_reward: ", task_reward)
            print("angle_reg_reward: ", angle_reg_reward)
            print("depth_reg_reward: ", depth_reg_reward)
        #     print("replan_reward: ", replan_reward)

    # def check_action_validity(self, action: Dict[str, Union[int, np.ndarray]]):
    #     cur_action = self.action_converter.convert(action)
    #     self.cur_position = [self.human_path_buffer[-1][0], self.human_path_buffer[-1][1]]

    #     if cur_action[0] == LOCAL_GOAL:
    #         # rescale to the map size
    #         cur_action[1][0] = cur_action[1][0] * self.obser_width
    #         cur_action[1][1] = cur_action[1][1] * self.obser_width

    #         is_goal_valid, goal, cone_center = self._check_predicted_goal(depth=cur_action[1][0], radius=cur_action[1][1])

    #         if is_goal_valid:
    #             self.path_planner.loadCone(cone_center=cone_center, 
    #                                         current_pos=self.cur_position,
    #                                         radius=cur_action[1][1],
    #                                         is_enabled=True)
    #             if not self._check_robot_path([self.cur_position[0], self.cur_position[1]], goal):
    #                 return 0
    #         else:
    #             return 0

    #     return 1

    # def _check_robot_path(self, start, end):
    #     cur_robot_path = self.path_planner.plan(cpp_utils.Point(start[0], start[1]), 
    #                                                      cpp_utils.Point(end[0], end[1]))
    #     cur_robot_path = [[pose.x, pose.y] for pose in cur_robot_path]

    #     if len(cur_robot_path) == 0:
    #         return False

    #     # connected to the nearest target（the global goal in this case）
    #     # because the cone only be used once, so the extended path won't be influenced by the cone
    #     if end[0] != self.global_goal[0] or end[1] != self.global_goal[1]:
    #         rest_robot_path = self.path_planner.plan(cpp_utils.Point(end[0], end[1]),
    #                                                     cpp_utils.Point(self.global_goal[0], self.global_goal[1]))
    #         rest_robot_path = [[pose.x, pose.y] for pose in rest_robot_path]

    #         if len(rest_robot_path) == 0:
    #             return False

    #     return True

            
    # def _check_predicted_goal(self, depth, radius):
    #     # cause before step, the robot direction is updated first
    #     cur_idx = int(self.time // self.time_resolution)
    #     speed_window = self.current_human_traj[cur_idx - self.speed_buffer_length + 1:cur_idx + 1]
    #     robot_direction = np.array([np.mean(np.array(speed_window[:, 3])),
    #                                np.mean(np.array(speed_window[:, 4]))]) 
    #     robot_direction = robot_direction / (robot_direction.dot(robot_direction)**0.5)

    #     cone_center = [
    #         self.cur_position[0] + depth * robot_direction[0],
    #         self.cur_position[1] + depth * robot_direction[1]
    #     ]
    #     vertices = []
    #     for i in range(2):
    #         x = cone_center[0] + radius * robot_direction[1] * math.cos(i * math.pi)
    #         y = cone_center[1] - radius * robot_direction[0] * math.cos(i * math.pi)
    #         vertices.append({'x': x, 'y': y})

    #     # Intersection of two perpendicular lines
    #     global_x = self.global_goal[0]
    #     global_y = self.global_goal[1]
    #     inter_x = ((global_y - vertices[0]['y']) * (vertices[1]['y'] - vertices[0]['y']) * (vertices[1]['x'] - vertices[0]['x']) +
    #                global_x * (vertices[1]['x'] - vertices[0]['x']) * (vertices[1]['x'] - vertices[0]['x']) + 
    #                vertices[0]['x'] * (vertices[1]['y'] - vertices[0]['y']) * (vertices[1]['y'] - vertices[0]['y'])) / ((vertices[1]['y'] - vertices[0]['y']) * (vertices[1]['y'] - vertices[0]['y']) + (vertices[1]['x'] - vertices[0]['x']) * (vertices[1]['x'] - vertices[0]['x']))
    #     inter_y = (vertices[0]['x'] - vertices[1]['x']) / (vertices[1]['y'] - vertices[0]['y']) * (inter_x - global_x) + global_y
        
    #     # select the nearest point to the global goal from the base edge of cone
    #     vector_0 = np.array([global_x - vertices[0]['x'], global_y - vertices[0]['y']])
    #     module_0 = vector_0.dot(vector_0) ** 0.5
    #     vector_1 = np.array([global_x - vertices[1]['x'], global_y - vertices[1]['y']])
    #     module_1 = vector_1.dot(vector_1) ** 0.5
    #     base_direction = np.array([robot_direction[1], -robot_direction[0]])
    #     cos_0 = vector_0.dot(base_direction) / module_0
    #     cos_1 = vector_1.dot(base_direction) / module_1

    #     try:
    #         if cos_0 * cos_1 > 0:
    #             # unilateral
    #             if abs(cos_0) < abs(cos_1):
    #                 # the vertex 0 is close to global goal
    #                 pred_position = self._avoidObstacles(vertices[0], vertices[1], vertices[0])
    #             else:
    #                 # the vertex 1 is close to global goal
    #                 pred_position = self._avoidObstacles(vertices[0], vertices[1], vertices[1])
    #         else:
    #             # bilateral
    #             pred_position = self._avoidObstacles(vertices[0], vertices[1], {'x':inter_x, 'y':inter_y})
    #     except Exception as e:
    #         # print('[Predictor] Fail to get the predicted goal: %s' % str(e))
    #         return False, None, None
        
    #     if pred_position is None:
    #          return False, None, None

    #     return True, pred_position, cone_center
    


