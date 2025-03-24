#!/usr/bin/python3
import sys
import os

# Add the workspace path to the PYTHONPATH
workspace_path = os.path.join(os.path.dirname(__file__) + "/../..")
sys.path.append(workspace_path)

from rl_replanner_train.train_env import SimulationWorld

print("Test gym environment")
print("=====================================")

# run environment
reward_weight = {
        'task': 1.0,
        'reg_angle': 0,
        'reg_depth': 0,
        'reg_replan': 0.05,
        'state': 2.0,
        'exp_factor': 1.0,
        'decay_factor': 0.98
    }
obser_width=5                # unit: meter
human_history_length=20
robot_prediction_length=100
speed_buffer_length=4
env = SimulationWorld(
    reward_weight=reward_weight,
    map_setting_file='/home/rosdev/ros2_ws/rl_replanner_train/maps/tb3_classic/turtlebot3_world.yaml',
    planner_setting_file='/home/rosdev/ros2_ws/cpp_utils/include/path_planner/planner_setting.yaml',
    # render_mode='ros',
    # render_real_time_factor=4,
    obser_width=obser_width,
    replay_traj_path='/home/rosdev/ros2_ws/rl_replanner_train/data',
    human_history_length=human_history_length,
    robot_prediction_length=robot_prediction_length,
    speed_buffer_length=speed_buffer_length
)

obs, info = env.reset()

print("Observation space:", env.observation_space)
print("Shape of observation space:", env.observation_space.shape)
print("Action space:", env.action_space)

step = 0
total_reward = 0
while True:
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward
    # print('Step:', step)
    # print('Action:', action)
    print('Observation:', obs)
    print('Reward:', reward)
    # print('Done:', terminated)
    # print('Info:', info)

    if terminated:
        print("Episode finished after {} timesteps".format(step + 1))
        obs, info = env.reset()
        print('===================== Total reward:', total_reward, '=====================')
        total_reward = 0
    
    step += 1

env.close()
