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

# Action Id
DO_NOTHING = 0
# GLOBAL_GOAL = 1
LOCAL_GOAL = 1


# simplest simulation world, version 2.0
class SimulationWorld(gym.Env):
    metadata = {"render_modes": ["ros", "none"],
                "render_real_time_factor": 1.0}

    def __init__(
            self, 
            reward_weight, 
            map_setting_file,
            planner_setting_file,
            render_mode=None, 
            obser_width=5, 
            map_resolution=0.05, 
            history_length=20, 
            replay_traj_path=None,
            decision_interval=1
        ):
        
        self.render_mode = render_mode

        # define observation space
        self.window_width_pixel = int(obser_width / map_resolution)            # TODO: training env can get the map resolution from the map setting file
        self.obser_width = obser_width
        self.history_length = history_length
        self.observation_space = spaces.Dict(
            {
                "partial_map": spaces.Box(0, 255, (1, self.window_width_pixel, self.window_width_pixel), np.uint8),
                "d_goal": spaces.Box(-0.5, 0.5, (2,), np.float32),
                "human_path": spaces.Box(-0.5, 0.5, (2 * history_length,), np.float32),
                "robot_path": spaces.Box(-0.5, 0.5, (2 * history_length,), np.float32),
                "last_action": spaces.Discrete(3),
            }
        )
        self.structure_obs = None

        # define action space
        parameterized_action_set = {
            DO_NOTHING: {},
            # GLOBAL_GOAL: {},
            LOCAL_GOAL: {
                'depth': [1e-3, np.sqrt(2)/2],
                'radius': [1e-3, np.sqrt(2)/2]
            }
        }
        self.action_converter = ActionConverter(parameterized_action_set)
        self.action_space = self.action_converter.gym_space_setting()
        self.current_action = (DO_NOTHING, [0, 0])
        self.is_action_ready = False

        # define reward
        self.reward_weight = reward_weight
        self.decay_factor = self.reward_weight['decay_factor'] # task reward decay factor
        self.decay_weight = [self.decay_factor ** i for i in reversed(range(self.history_length))] 
        self.decay_weight = np.array(self.decay_weight) * (1 - self.decay_factor) / (1 - self.decay_factor ** (self.history_length))
        self.exp_factor = self.reward_weight['exp_factor']     # when error is 0.5 (which is half length of edge of partial square map), the approximate reward is 0.3
        self.len_threshold = 1/4  # half length of the partial square map
        self.reward = 0.0

        ################################################################################################################################
        # define simulation world parameters
        self.time = 0
        self.decision_interval = decision_interval                       # seconds
        self.speed_buffer_length = int(self.history_length / 5)          # TODO: time length is self.history_length * self.time_resolution (change the test simulation env also)

        # human path
        self.replay_traj_files = glob.glob(replay_traj_path + '/*.txt')
        self.current_human_traj = None
        self.time_resolution = 0.1            # collect rate is 10Hz
        self.path_resolution = 0.05 / 4
        self.human_path_buffer = []

        # robot path
        self.current_robot_path = None
        self.robot_path_buffer = []

        # global goal
        self.global_goal = None

        # partial map
        self.partial_map = None

        # load occupancy grid map
        res, self.global_costmap = cpp_utils.loadMap(map_setting_file)
        self.map_resolution = self.global_costmap.resolution
        if res != cpp_utils.LOAD_MAP_STATUS.LOAD_MAP_SUCCESS:
            raise ValueError("Failed to load map setting file.")
        
        # load planner
        self.path_planner = cpp_utils.PathPlanner()
        self.path_planner.configure(self.global_costmap, planner_setting_file)

        # temporary variables for action
        self.cur_position = None
        self.cone_center = None
        self.pred_goal = None
        self.robot_direction = None

        ################################################################################################################################
        # render setting
        if self.render_mode == "ros":
            rclpy.init()

            self.inflated_distance = self.path_planner.inflated_distance

            # render_node
            self.render_ros = rosRender()
            self.render_ros.on_configure(self.global_costmap)
            self.render_ros.on_activate()

            # launch rviz
            # Create the LauchService and feed the LaunchDescription obj. to it.
            launchService = LaunchService()
            launchService.include_launch_description(generate_rviz_launch_description())
            self.process = Process(target=launchService.run)
            # The daemon process is terminated automatically before the main program exits,
            # to avoid leaving orphaned processes running
            self.process.daemon = True
            self.process.start()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.time = 0
        self.current_action = (DO_NOTHING, [0, 0])

        # load human trajectory
        traj_file = np.random.choice(self.replay_traj_files)
        self.current_human_traj = np.loadtxt(traj_file)
        self.global_goal = [self.current_human_traj[-1][0], self.current_human_traj[-1][1]]
        self.human_path_buffer = []

        # time elapsed
        self._plan_robot_path([self.current_human_traj[0][0], self.current_human_traj[0][1]], self.global_goal)
        
        if len(self.current_robot_path) == 0:
            raise ValueError("[SimulationWorld] Failed to find a path from start to goal.")
        
        self.human_path_buffer.append([self.current_human_traj[0][0], self.current_human_traj[0][1]])
        idx = 1
        while len(self.human_path_buffer) < self.history_length:
            idx += 1
            if idx >= len(self.current_human_traj):
                raise ValueError("[SimulationWorld] Human total path is too short.")

            x = self.current_human_traj[idx][0]
            y = self.current_human_traj[idx][1]
            distance = ((x - self.human_path_buffer[-1][0]) ** 2 + (y - self.human_path_buffer[-1][1]) ** 2) ** 0.5
            if distance >= self.path_resolution:
                self.human_path_buffer.append([x, y])


        self.time += self.time_resolution * idx

        # get observation
        # move robot to the decided position
        self._get_obs()

        # get info
        info = self._get_info()

        # TODO: add render function
        self._get_robot_direction()
        self.render()

        return self.structure_obs, info

    def step(self, raw_action: Dict[str, Union[int, np.ndarray]]):
        self.current_action = self.action_converter.convert(raw_action)

        terminated = False
        # apply action
        # direction vector is average velocity calculated from past trajectory
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
                self._plan_robot_path([self.cur_position[0], self.cur_position[1]], self.pred_goal)
            else:
                terminated = True
        elif self.current_action[0] == DO_NOTHING:
            pass
        
        if len(self.current_robot_path) == 0:
            print("[SimulationWorld] Failed to find a path from start to goal.")
            terminated = True

        if terminated:
            end_reward = -1
        elif self._get_human_path():
            terminated = True
            end_reward = 1
        else:
            self.time += self.decision_interval
            end_reward = 0

        # get observation
        self._get_obs(is_terminal=terminated)

        # calculate reward
        self.calculate_reward(is_terminal=terminated)
        self.reward += end_reward * self.reward_weight['state']

        # get info
        info = self._get_info()

        # TODO: add render function
        self.render()

        return self.structure_obs, self.reward, terminated, False, info

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")

        # close the environment
        if self.render_mode == "ros":
            self.render_ros.on_deactivate()
            self.render_ros.on_shutdown()

            parent = psutil.Process(self.process.pid)
            for child in parent.children(recursive=True):
                child.kill()
            parent.kill()

            rclpy.shutdown()


    # after calling _get_obs, the flag 'is_obs_ready_' will be set to false
    def _get_obs(self, is_terminal=False):
        self.structure_obs = {}
        if not is_terminal:
            self.structure_obs["robot_path"] = self._get_robot_path()
            self.structure_obs["human_path"] = self.human_path_buffer
            self.structure_obs["partial_map"] = self._get_partial_map()
            self.structure_obs["d_goal"] = self.global_goal
            self.structure_obs["last_action"] = int(self.current_action[0])
            self._regularization()
        else:
            self.structure_obs["partial_map"] = np.zeros((1, self.window_width_pixel, self.window_width_pixel), dtype=np.uint8)
            self.structure_obs["human_path"] = np.zeros((2 * self.history_length,), dtype=np.float32)
            self.structure_obs["robot_path"] = np.zeros((2 * self.history_length,), dtype=np.float32)
            self.structure_obs["d_goal"] = np.zeros((2,), dtype=np.float32)
            self.structure_obs["last_action"] = int(self.current_action[0])

        return self.structure_obs
    
    def _get_robot_direction(self):
        # direction vector is average velocity calculated from past trajectory
        cur_idx = int(self.time // self.time_resolution)
        speed_window = self.current_human_traj[cur_idx - self.speed_buffer_length + 1:cur_idx + 1]
        self.robot_direction = np.array([np.mean(np.array(speed_window[:, 3])),
                                   np.mean(np.array(speed_window[:, 4]))]) 
        self.robot_direction = self.robot_direction / (self.robot_direction.dot(self.robot_direction)**0.5)

    def _plan_robot_path(self, start, end):
        self.current_robot_path = self.path_planner.plan(cpp_utils.Point(start[0], start[1]), 
                                                         cpp_utils.Point(end[0], end[1]))
        self.current_robot_path = [[pose.x, pose.y] for pose in self.current_robot_path]

        # debug
        print("robot path length: ", len(self.current_robot_path))

        # extend the robot path to include historical human path
        if len(self.human_path_buffer) == self.history_length:
            self.current_robot_path = self.human_path_buffer + self.current_robot_path
            
            # debug
            print("robot path length after extension: ", len(self.current_robot_path))

        self.current_index = self.history_length

    def _get_human_path(self):
        # update the human path buffer
        idx = int(self.time // self.time_resolution)
        idx_len = int(self.decision_interval // self.time_resolution)

        for i in range(idx + 1, idx + idx_len + 1):
            if i >= len(self.current_human_traj):
                return True

            x = self.current_human_traj[i][0]
            y = self.current_human_traj[i][1]
            distance = ((x - self.human_path_buffer[-1][0]) ** 2 + (y - self.human_path_buffer[-1][1]) ** 2) ** 0.5
            if distance >= self.path_resolution:
                self.human_path_buffer.pop(0)
                self.human_path_buffer.append([x, y])
        
        return False


    def _get_robot_path(self):
        k = 5   # include the boundary
        cur_robot_path_length = len(self.current_robot_path)
        start_idx = max(0, self.current_index - self.history_length - k)
        end_idx = min(cur_robot_path_length, self.current_index + self.history_length + k)
        min_ds = np.inf
        min_de = np.inf
        idx_ = start_idx
        start = self.human_path_buffer[0]
        end = self.human_path_buffer[-1]
        for i in range(start_idx, end_idx):
            robot_pose = self.current_robot_path[i]
            ds = ((robot_pose[0] - start[0]) ** 2 + (robot_pose[1] - start[1]) ** 2) ** 0.5
            de = ((robot_pose[0] - end[0]) ** 2 + (robot_pose[1] - end[1]) ** 2) ** 0.5
            if ds < min_ds:
                min_ds = ds
                idx_ = i
            if de < min_de:
                min_de = de
                self.current_index = i

        self.robot_path_buffer = self.current_robot_path[idx_: idx_ + self.history_length]
        if cur_robot_path_length < idx_ + self.history_length:
            # append the terminal poses repeatedly
            for i in range(cur_robot_path_length, idx_ + self.history_length):
                self.robot_path_buffer.append(self.current_robot_path[-1])
        
        return self.robot_path_buffer

    def _get_partial_map(self):
        # Extract the partial map
        self.partial_map = self.global_costmap.getPartialCostmap(self.human_path_buffer[-1][0],
                                                                self.human_path_buffer[-1][1],
                                                                self.obser_width, 
                                                                self.obser_width)
        size_x = self.partial_map.size_x
        size_y = self.partial_map.size_y
        data = np.array(self.partial_map.data, dtype=np.uint8) # raw data from costmap (0-255) TODO: change the old simulation env

        map_2d = data.reshape((size_y, size_x)) 
        map_2d = np.flip(map_2d, axis=0)                       # TODO: check if the shape is correct

        # debug 
        # # print map_2d to txt file
        # np.savetxt("map_2d_y_x.txt", data.reshape((size_y, size_x)), fmt='%4d', delimiter=' ')
        # np.savetxt("map_2d_x_y.txt", data.reshape((size_x, size_y)), fmt='%4d', delimiter=' ')

        map_2d = np.expand_dims(map_2d, axis=0) # channel first

        # Validate with Gym space
        try:
            assert map_2d.shape == self.observation_space["partial_map"].shape
            assert map_2d.dtype == self.observation_space["partial_map"].dtype
        except AssertionError as e:
            print("AssertionError: ", e)
            print("map_2d: ", map_2d.shape)
            print("self.observation_space['partial_map']: ", self.observation_space["partial_map"].shape)
            raise e

        return map_2d

    def _regularization(self):
        cur_x = self.structure_obs["human_path"][-1][0]
        cur_y = self.structure_obs["human_path"][-1][1]

        human_path_np = np.array([[pose[0] - cur_x, pose[1] - cur_y] for pose in self.structure_obs["human_path"]])
        robot_path_np = np.array([[pose[0] - cur_x, pose[1] - cur_y] for pose in self.structure_obs["robot_path"]])

        flattened_human_path = human_path_np.flatten()
        flattened_robot_path = robot_path_np.flatten()

        # Normalize the path
        flattened_human_path = flattened_human_path / self.obser_width
        flattened_robot_path = flattened_robot_path / self.obser_width

        self.structure_obs["human_path"] = np.clip(flattened_human_path, -0.5, 0.5).astype(np.float32)
        self.structure_obs["robot_path"] = np.clip(flattened_robot_path, -0.5, 0.5).astype(np.float32)

        d_goal_np = np.array([self.structure_obs["d_goal"][0] - cur_x, self.structure_obs["d_goal"][1] - cur_y])
        # Normalize the goal
        d_goal_np = d_goal_np / self.obser_width
        self.structure_obs["d_goal"] = np.clip(d_goal_np, -0.5, 0.5).astype(np.float32)

        # Validate with Gym space
        try:
            assert self.structure_obs["human_path"].shape == self.observation_space["human_path"].shape
            assert self.structure_obs["human_path"].dtype == self.observation_space["human_path"].dtype
            assert self.structure_obs["robot_path"].shape == self.observation_space["robot_path"].shape
            assert self.structure_obs["robot_path"].dtype == self.observation_space["robot_path"].dtype
            assert self.structure_obs["d_goal"].shape == self.observation_space["d_goal"].shape
            assert self.structure_obs["d_goal"].dtype == self.observation_space["d_goal"].dtype
        except AssertionError as e:
            print("AssertionError: ", e)
            print("flattened_human_path: ", self.structure_obs["human_path"].shape)
            print("flattened_robot_path: ", self.structure_obs["robot_path"].shape)
            print("d_goal_np: ", self.structure_obs["d_goal"].shape)
            print("self.observation_space['human_path']: ", self.observation_space["human_path"].shape)
            print("self.observation_space['robot_path']: ", self.observation_space["robot_path"].shape)
            print("self.observation_space['d_goal']: ", self.observation_space["d_goal"].shape)
            raise e
        
    def _get_predicted_goal(self, depth, radius):
        # position
        self.cur_position = [self.human_path_buffer[-1][0], self.human_path_buffer[-1][1]]  

        # debug 
        # these two positions are not the same !!!
        # print("current position from idx: ", self.cur_position)
        # print("current position from buffer: ", [self.human_path_buffer[-1][0], self.human_path_buffer[-1][1]])

        self.cone_center = [
            self.cur_position[0] + depth * self.robot_direction[0],
            self.cur_position[1] + depth * self.robot_direction[1]
        ]
        vertices = []
        for i in range(2):
            x = self.cone_center[0] + radius * self.robot_direction[1] * math.cos(i * math.pi)
            y = self.cone_center[1] - radius * self.robot_direction[0] * math.cos(i * math.pi)
            vertices.append({'x': x, 'y': y})

        # Intersection of two perpendicular lines
        global_x = self.global_goal[0]
        global_y = self.global_goal[1]
        inter_x = ((global_y - vertices[0]['y']) * (vertices[1]['y'] - vertices[0]['y']) * (vertices[1]['x'] - vertices[0]['x']) +
                   global_x * (vertices[1]['x'] - vertices[0]['x']) * (vertices[1]['x'] - vertices[0]['x']) + 
                   vertices[0]['x'] * (vertices[1]['y'] - vertices[0]['y']) * (vertices[1]['y'] - vertices[0]['y'])) / ((vertices[1]['y'] - vertices[0]['y']) * (vertices[1]['y'] - vertices[0]['y']) + (vertices[1]['x'] - vertices[0]['x']) * (vertices[1]['x'] - vertices[0]['x']))
        inter_y = (vertices[0]['x'] - vertices[1]['x']) / (vertices[1]['y'] - vertices[0]['y']) * (inter_x - global_x) + global_y
        
        # select the nearest point to the global goal from the base edge of cone
        vector_0 = np.array([global_x - vertices[0]['x'], global_y - vertices[0]['y']])
        module_0 = vector_0.dot(vector_0) ** 0.5
        vector_1 = np.array([global_x - vertices[1]['x'], global_y - vertices[1]['y']])
        module_1 = vector_1.dot(vector_1) ** 0.5
        base_direction = np.array([self.robot_direction[1], -self.robot_direction[0]])
        cos_0 = vector_0.dot(base_direction) / module_0
        cos_1 = vector_1.dot(base_direction) / module_1

        try:
            if cos_0 * cos_1 > 0:
                # unilateral
                if abs(cos_0) < abs(cos_1):
                    # the vertex 0 is close to global goal
                    pred_position = self._avoidObstacles(vertices[0], vertices[1], vertices[0])
                else:
                    # the vertex 1 is close to global goal
                    pred_position = self._avoidObstacles(vertices[0], vertices[1], vertices[1])
            else:
                # bilateral
                pred_position = self._avoidObstacles(vertices[0], vertices[1], {'x':inter_x, 'y':inter_y})
        except Exception as e:
            print('[Predictor] Fail to get the predicted goal: %s' % str(e))
            return False
        
        if pred_position is None:
             return False

        # angle
        # angle_depth = (1 - math.exp(-self.pred_decay_factor * depth)) / self.pred_decay_factor
        # avr_angle_vel = np.mean(np.array(self.actual_states.w))
        # avr_angle_vel = avr_angle_vel / abs(avr_angle_vel)
        # pred_theta = self.actual_states.theta[-1] + angle_depth * avr_angle_vel

        # Becasue the global planner, Navfn, doesn't consider the orientation of the robot along the path 
        # and the parameter "use_final_approach_orientation" is set to true,
        # the last pose of planned path is always set to the approach orientation. 
        # So we don't need to consider the orientation of the predict goal.

        self.pred_goal = copy.copy(pred_position)

        return True
    
    def _isCollided(self, point):
        if (point[0] < self.global_costmap.origin_x or point[1] < self.global_costmap.origin_y):
            return True
        
        mx = int((point[0] - self.global_costmap.origin_x) / self.map_resolution)
        my = int((point[1] - self.global_costmap.origin_y) / self.map_resolution)

        if (mx < self.global_costmap.size_x and my < self.global_costmap.size_y) and self.global_costmap.getCost(mx, my) < 252:
            return False
        else:
            return True

    # Return the collision-free point closest to the target on the line segment from vertex_0 to vertex_1
    def _avoidObstacles(self, vertex_0, vertex_1, target):
        module = ((vertex_1['x'] - vertex_0['x']) ** 2 + (vertex_1['y'] - vertex_0['y']) ** 2) ** 0.5
        if target['x'] == vertex_0['x'] and target['y'] == vertex_0['y']:
            dir_x = (vertex_1['x'] - vertex_0['x']) / module
            dir_y = (vertex_1['y'] - vertex_0['y']) / module
            distance = 0
            p_ = [target['x'] + distance * dir_x, target['y'] + distance * dir_y]
            while self._isCollided(p_):
                distance += self.map_resolution
                if distance > module:
                    print('[Predictor] The lien segment is in the obstacles')
                    return None
                else:
                    p_ = [target['x'] + distance * dir_x, target['y'] + distance * dir_y]
            return p_
        elif target['x'] == vertex_1['x'] and target['y'] == vertex_1['y']:
            dir_x = (vertex_0['x'] - vertex_1['x']) / module
            dir_y = (vertex_0['y'] - vertex_1['y']) / module
            distance = 0
            p_ = [target['x'] + distance * dir_x, target['y'] + distance * dir_y]
            while self._isCollided(p_):
                distance += self.map_resolution
                if distance > module:
                    print('[Predictor] The lien segment is in the obstacles')
                    return None
                else:
                    p_ = [target['x'] + distance * dir_x, target['y'] + distance * dir_y]
            return p_
        else:
            dir_x = (vertex_0['x'] - vertex_1['x']) / module
            dir_y = (vertex_0['y'] - vertex_1['y']) / module
            distance = 0
            module_0 = ((target['x'] - vertex_0['x']) ** 2 + (target['y'] - vertex_0['y']) ** 2) ** 0.5
            module_1 = module - module_0
            p_0 = [target['x'] + distance * dir_x, target['y'] + distance * dir_y]
            p_1 = p_0
            while True:
                if distance <= module_0 and not self._isCollided(p_0):
                    return p_0
                if distance <= module_1 and not self._isCollided(p_1):
                    return p_1
                distance += self.map_resolution
                if distance > module_0 and distance > module_1:
                    print('[Predictor] The lien segment is in the obstacles')
                    return None
                elif distance > module_0 and distance <= module_1:
                    p_1 = [target['x'] - distance * dir_x, target['y'] - distance * dir_y]
                elif distance <= module_0 and distance > module_1:
                    p_0 = [target['x'] + distance * dir_x, target['y'] + distance * dir_y]
                else:
                    p_0 = [target['x'] + distance * dir_x, target['y'] + distance * dir_y]
                    p_1 = [target['x'] - distance * dir_x, target['y'] - distance * dir_y]

    def calculate_reward(self, is_terminal=False):
        # task reward
        if not is_terminal:
            exp_error = np.exp(- self.exp_factor * np.linalg.norm((self.structure_obs['human_path'].reshape((-1,2)) - self.structure_obs['robot_path'].reshape((-1,2))), axis=1))
            # debug
            # print("max error: ", np.max(np.linalg.norm((self.structure_obs['human_path'].reshape((-1,2)) - self.structure_obs['robot_path'].reshape((-1,2))), axis=1)))
            # print("exp_error: ", exp_error)
            # print("decay_weight: ", np.round(self.decay_weight, 2))
            # print("decay_weight sum is: ", np.sum(self.decay_weight))
            task_reward = self.decay_weight.dot(exp_error) * self.reward_weight['task']
        else:
            task_reward = 0.0

        # regularization reward
        # radius / depth
        if self.current_action[0] == LOCAL_GOAL:
            angle_reg_reward = -np.arctan(self.current_action[1][1] / self.current_action[1][0]) / (np.pi / 2) * self.reward_weight['reg_angle']
        else:
            angle_reg_reward = 0.0

        # depth
        if self.current_action[0] == LOCAL_GOAL and self.current_action[1][0] < self.len_threshold:
            depth_reg_reward = (self.current_action[1][0] - self.len_threshold) / self.len_threshold * self.reward_weight['reg_depth']
        else:
            depth_reg_reward = 0.0
        
        # replan
        if self.current_action[0] == DO_NOTHING:
            replan_reward = 0.0
        else:
            replan_reward = -1 * self.reward_weight['reg_replan']
        
        self.reward = task_reward + angle_reg_reward + depth_reg_reward + replan_reward
        
        # debug
        # print("============== reward terms ==============")
        # print("task_reward: ", task_reward)
        # print("angle_reg_reward: ", angle_reg_reward)
        # print("depth_reg_reward: ", depth_reg_reward)
        # print("replan_reward: ", replan_reward)

        return self.reward

    def _get_info(self):
        return {}
    
    def render(self):
        if self.render_mode == "ros":
            # observation
            self.render_ros.pub_partial_map(self.partial_map)
            self.render_ros.pub_local_human_path(self.human_path_buffer, self.robot_direction)
            self.render_ros.pub_local_robot_path(self.robot_path_buffer)
            self.render_ros.pub_global_goal(self.global_goal)

            # simulation setup
            if self.current_action[0] == LOCAL_GOAL:
                self.render_ros.pub_global_map_with_cone(self.cur_position, self.cone_center, 
                                                         self.current_action[1][1], 
                                                         inflated_distance=self.inflated_distance)
            else:
                self.render_ros.pub_global_map()
            
            self.render_ros.pub_human_path(self.current_human_traj, self.path_resolution)
            self.render_ros.pub_robot_path(self.current_robot_path)

            # sleep this thread to make the simulation real-time
            time.sleep(self.decision_interval)
            





