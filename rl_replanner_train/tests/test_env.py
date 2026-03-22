#!/usr/bin/python3
import sys
import os
import time

# Add the workspace path to the PYTHONPATH
workspace_path = os.path.join(os.path.dirname(__file__) + "/../..")
print("Workspace path:", workspace_path)
sys.path.append(workspace_path)

from rl_replanner_train.train_env import TrainEnv

print("Test gym environment")
print("=====================================")

# run environment
reward_weight = {
        'task': 1.0,
        'replan_punishment': 0.3,
        'reg_angle_factor_a': 0.0,
        'reg_angle_factor_b': 3.0,
        'reg_angle_factor_k': 0.02,
        # 'reg_depth_factor_b': 3.0,
        # 'reg_depth_init_portion': 2.0,
        'state': 2.0,
        'exp_factor': 1.0,
        'decay_factor': 0.98,
        'replan_punishment': 1.0   # 补充
    }
obser_width=12                # unit: meter
human_history_length=20
robot_prediction_length=100
speed_buffer_length=4
env = TrainEnv(
    reward_weight=reward_weight,
    # map_setting_file='./rl_replanner_train/maps/sim_maps/turtlebot3_world.yaml',
    map_setting_file='./rl_replanner_train/maps/sim_maps/circle_clutter.yaml',
    path_planner_setting_file='./cpp_utils/include/path_planner/planner_setting.yaml',
    traj_planner_setting_file="./cpp_utils/include/teb_local_planner/teb_params.yaml",
    render_mode='ros',
    render_real_time_factor=10,
    obser_width=obser_width,
    replay_traj_path='./rl_replanner_train/data',
    human_history_length=human_history_length,
    robot_prediction_length=robot_prediction_length,
    speed_buffer_length=speed_buffer_length,
    use_generator=True,  # Set to True if you want to use the generator
    intention_domain_type="ellipse",  # Options: "cone", "ellipse", "rectangle"
)

obs, info = env.reset()

print("Observation space:", env.observation_space)
print("Shape of observation space:", env.observation_space.shape)
print("Action space:", env.action_space)

step = 0
total_reward = 0
while True:
    action = env.action_space.sample()

    # print('Action:', action)
    # action = {
    #     'id': 1,
    #     'params0': [],
    #     'params1': [0.1, 0.2, -0.5],
    # }
    # print('Action:', action)

    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward
    # print('Step:', step)
    # print('Action:', action)
    # print('Observation:', obs)
    # print('Reward:', reward)W
    # print('Done:', terminated)
    # print('Info:', info)

    time.sleep(0.5)  # Sleep for a short time to slow down the loop for better visualization

    if terminated:
        print("Episode finished after {} timesteps".format(step + 1))
        obs, info = env.reset()
        print('===================== Total reward:', total_reward, '=====================')
        total_reward = 0
    
    step += 1

env.close()
