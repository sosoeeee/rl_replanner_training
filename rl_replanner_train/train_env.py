import glob
import time
import numpy as np
from typing import Dict, Union
import cpp_utils

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
            map_setting_file_for_planner = None
        ):
        # addtional parameters
        self.traj_planner_setting_file = traj_planner_setting_file
        if map_setting_file_for_planner is not None:
            self.map_setting_file_for_planner = map_setting_file_for_planner
        else:
            self.map_setting_file_for_planner = None

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
        map_name = self.map_setting_file.split('/')[-1].split('.')[0]
        self.replay_traj_files = glob.glob(self.replay_traj_path + '/' + map_name + '/collected_paths/*.txt')

        # TODO: initialize the human traj generator
        if self.use_generator:
            self.traj_generator = cpp_utils.TrajGenerator()
            if self.map_setting_file_for_planner is not None:
                self.traj_generator.initialize(
                    map_file=self.map_setting_file_for_planner,
                    planner_file=self.traj_planner_setting_file,
                    path_resolution=self.path_resolution,
                    time_resolution=self.time_resolution,
                ) 
            else:
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

        self.global_goal = [self.current_human_traj[-1][0], self.current_human_traj[-1][1]]

        # TODO: generate a human trajectory. Its start and end point are the same as the trajectory loaded from the file
        if self.use_generator:
            start_point = cpp_utils.Point(self.current_human_traj[0][0], self.current_human_traj[0][1])
            end_point = cpp_utils.Point(self.current_human_traj[-1][0], self.current_human_traj[-1][1])

            # random exchange the start and end point
            # if np.random.rand() > 0.5:
            #     start_point, end_point = end_point, start_point

            # generated_traj = self.traj_generator.sampleTraj(start=start_point, end=end_point)
            generated_traj = self.traj_generator.sampleTrajLoop(start=start_point, end=end_point)
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
    
    # When trianing, allow robot change its aciton when current one is invalid
    def _interact(self):
        # debug
        # if self.render_mode == "ros":
        #     print("current action: ", self.current_action)

        # This function is used to interact with the environment
        self.cur_position = [self.human_path_buffer[-1][0], self.human_path_buffer[-1][1]] 

        terminated = False

        # apply action
        # direction vector is average velocity calculated from past trajectory
        is_valid = True
        self._get_robot_direction()

        if self.current_action[0] == LOCAL_GOAL:
            # rescale to the map size
            self.current_action[1][0] = self.current_action[1][0] * self.obser_width
            self.current_action[1][1] = self.current_action[1][1] * self.obser_width

            if self._get_predicted_goal(depth=self.current_action[1][0], radius=self.current_action[1][1]):
                self.path_planner.loadCone(cone_center=self.cone_center, 
                                            current_pos=self.cur_position,
                                            radius=self.current_action[1][1],
                                            is_enabled=True)
                if not self._plan_robot_path([self.cur_position[0], self.cur_position[1]], self.pred_goal):
                    is_valid = False
                # use point on robot path as the start point
                # self._plan_robot_path([self.current_robot_path[self.robot_closest_idx][0], self.current_robot_path[self.robot_closest_idx][1]], self.pred_goal)
            else:
                is_valid = False

        if not is_valid:
            # time won't elapse if the action is invalid
            end_reward = -1
        else:
            end_reward = 0  
            if self._get_human_path():
                terminated = True
                end_reward = 1
            else:
                self.time += self.decision_interval

        return terminated, end_reward
    
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
            r_p = self.robot_path_buffer[:eval_length]
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
                # angle_reg_reward = -np.arctan(self.current_action[1][1] / self.current_action[1][0]) / (np.pi / 2) * self.reward_weight['reg_angle']
                norm_angle = np.arctan(self.current_action[1][1] / self.current_action[1][0]) / (np.pi / 2)
                angle_reg_reward = self.reward_weight['reg_angle'] * (np.log(1 - norm_angle)) / (1 - norm_angle) 
                # print("angle_reg_reward: ", angle_reg_reward)
            else:
                angle_reg_reward = 0.0
        else:
            angle_reg_reward = 0.0
        
        self.reward = task_reward + angle_reg_reward + end_reward * self.reward_weight['state']
        
        # debug
        if self.render_mode == "ros":
            print("============== reward terms ==============")
            print("end_reward: ", end_reward * self.reward_weight['state'])
            print("task_reward: ", task_reward)
            print("angle_reg_reward: ", angle_reg_reward)
        #     print("replan_reward: ", replan_reward)
    


