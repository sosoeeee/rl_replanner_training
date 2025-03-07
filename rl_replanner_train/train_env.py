import numpy as np
from typing import Tuple, Dict, Union
import gymnasium as gym
from gymnasium import spaces

from rl_replanner_train.action_converter import ActionConverter

# Action Id
DO_NOTHING = 0
# GLOBAL_GOAL = 1
LOCAL_GOAL = 1


# simplest simulation world, version 2.0
class SimulationWorld(gym.Env):
    metadata = {"render_modes": ["human"],
                "render_real_time_factor": 1.0}

    def __init__(
            self, 
            reward_weight, 
            render_mode=None, 
            obser_width=5, 
            map_resolution=0.05, 
            history_length=20, 
            replay_traj_path=None,
            decision_interval=1
        ):
        
        self.render_mode = render_mode
        self.replay_traj_path = replay_traj_path
        self.current_human_traj = None
        self.decision_interval = decision_interval        # seconds

        # define observation space
        self.window_width_pixel = int(obser_width / map_resolution) - 1
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
        self.last_action = None
        self.is_action_ready = False

        self.reward_weight = reward_weight
        self.decay_factor = self.reward_weight['decay_factor'] # task reward decay factor
        self.decay_weight = [self.decay_factor ** i for i in reversed(range(self.history_length))] 
        self.decay_weight = np.array(self.decay_weight) * (1 - self.decay_factor) / (1 - self.decay_factor ** (self.history_length))
        self.exp_factor = self.reward_weight['exp_factor']     # when error is 0.5 (which is half length of edge of partial square map), the approximate reward is 0.3
        self.len_threshold = 1/4  # half length of the partial square map
        self.reward = 0.0

        ################################################################################################################################
        # load occupancy grid map

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # load human trajectory
        traj_file = np.random.choice(self.replay_traj_files)
        self.current_human_traj = np.loadtxt(traj_file)

        # reset robot here

        # get observation
        # move robot to the decided position
        self._get_obs()

        # get info
        info = self._get_info()

        # TODO: add render function

        return self.structure_obs, info

    def step(self, raw_action: Dict[str, Union[int, np.ndarray]]):
        self.current_action = self.action_converter.convert(raw_action)

        # apply action

        # check if the episode is terminated
        terminated = False

        # get observation
        self._get_obs()

        # calculate reward
        self.calculate_reward()

        # get info
        info = self._get_info()

        # TODO: add render function

        # update last action
        self.last_action = self.current_action

        return self.structure_obs, self.reward, terminated, False, info

    def close(self):
        print("Closing " + self.__class__.__name__ + " environment.")

        # close the environment


    # after calling _get_obs, the flag 'is_obs_ready_' will be set to false
    def _get_obs(self, is_terminal=False):
        if not is_terminal:
            # TODO: get raw observation

            self.structure_obs = {}
            self.structure_obs["partial_map"] = self._occupancy_grid_to_greyscale(cur_obs.local_map)
            self.structure_obs["human_path"], self.structure_obs["robot_path"], self.structure_obs["d_goal"] = self._regularization(cur_obs.human_path, cur_obs.robot_path, cur_obs.global_goal)
            self.structure_obs["last_action"] = int(self.current_action[0])
        else:
            self.structure_obs = {}
            self.structure_obs["partial_map"] = np.zeros((1, self.window_width_pixel, self.window_width_pixel), dtype=np.uint8)
            self.structure_obs["human_path"] = np.zeros((2 * self.history_length,), dtype=np.float32)
            self.structure_obs["robot_path"] = np.zeros((2 * self.history_length,), dtype=np.float32)
            self.structure_obs["d_goal"] = np.zeros((2,), dtype=np.float32)
            self.structure_obs["last_action"] = int(self.current_action[0])

        return self.structure_obs

    def _occupancy_grid_to_greyscale(self, occupancy_grid):
        # Extract the data and reshape into 2D
        data = np.array(occupancy_grid.data, dtype=np.int8)  # raw data from OccupancyGrid
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        map_2d = data.reshape((height, width))

        # Map values to grayscale (0-255)
        # Unknown (-1) → 255, Free (0) → 0, Occupied (100) → 255
        map_2d = np.where(map_2d == -1, 255, map_2d)  # Map unknowns
        map_2d = np.clip(map_2d * 2.55, 0, 255).astype(np.uint8)  # Scale [0, 100] to [0, 255]

        # # Resize to (obser_width_pixel, obser_width_pixel)
        # resized_map = resize(map_2d, (obser_width_pixel, obser_width_pixel), interpolation=INTER_NEAREST)

        # add channel dimension
        map_2d = np.expand_dims(map_2d, axis=0) # channel first

        # Validate with Gym space
        assert map_2d.shape == self.observation_space["partial_map"].shape
        assert map_2d.dtype == self.observation_space["partial_map"].dtype

        return map_2d
    
    def _regularization(self, human_path, robot_path, d_goal):
        cur_x = human_path.poses[-1].pose.position.x
        cur_y = human_path.poses[-1].pose.position.y

        human_path_np = np.array([[pose.pose.position.x - cur_x, pose.pose.position.y - cur_y] for pose in human_path.poses])
        robot_path_np = np.array([[pose.pose.position.x - cur_x, pose.pose.position.y - cur_y] for pose in robot_path.poses])

        flattened_human_path = human_path_np.flatten()
        flattened_robot_path = robot_path_np.flatten()

        # Clip values to fit within [-obser_width, obser_width]
        flattened_human_path = np.clip(flattened_human_path, -0.5, 0.5).astype(np.float32)
        flattened_robot_path = np.clip(flattened_robot_path, -0.5, 0.5).astype(np.float32)

        d_goal_np = np.array([d_goal.pose.position.x - cur_x, d_goal.pose.position.y - cur_y])
        d_goal_np = np.clip(d_goal_np, -0.5, 0.5).astype(np.float32)

        # Validate with Gym space
        try:
            assert flattened_human_path.shape == self.observation_space["human_path"].shape
            assert flattened_human_path.dtype == self.observation_space["human_path"].dtype
            assert flattened_robot_path.shape == self.observation_space["robot_path"].shape
            assert flattened_robot_path.dtype == self.observation_space["robot_path"].dtype
            assert d_goal_np.shape == self.observation_space["d_goal"].shape
            assert d_goal_np.dtype == self.observation_space["d_goal"].dtype
        except AssertionError as e:
            print("AssertionError: ", e)
            print("flattened_human_path: ", flattened_human_path.shape)
            print("flattened_robot_path: ", flattened_robot_path.shape)
            print("d_goal_np: ", d_goal_np.shape)
            print("self.observation_space['human_path']: ", self.observation_space["human_path"].shape)
            print("self.observation_space['robot_path']: ", self.observation_space["robot_path"].shape)
            print("self.observation_space['d_goal']: ", self.observation_space["d_goal"].shape)
            raise e

        return flattened_human_path, flattened_robot_path, d_goal_np

    
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






