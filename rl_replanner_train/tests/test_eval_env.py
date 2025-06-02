#!/usr/bin/python3
import sys
import os
import time

# Add the workspace path to the PYTHONPATH
workspace_path = os.path.join(os.path.dirname(__file__) + "/../..")
sys.path.append(workspace_path)

from rl_replanner_train.eval_env import EvalEnv

print("Test Eval Environment")
print("=====================================")

# run environment
reward_weight = {
        'task': 1.0,
        'state': 2.0,
        'exp_factor': 1.0,
        'decay_factor': 0.98
    }
obser_width = 5  # 单位：米
human_history_length = 20
robot_prediction_length = 100
speed_buffer_length = 4

# 初始化 EvalEnv
env = EvalEnv(
    reward_weight=reward_weight,
    map_setting_file='./rl_replanner_train/maps/tb3_classic/turtlebot3_world_3.yaml',
    path_planner_setting_file='./cpp_utils/include/path_planner/planner_setting.yaml',
    render_mode='ros',
    render_real_time_factor=10,
    obser_width=obser_width,
    replay_traj_path='./rl_replanner_train/data/',
    human_history_length=human_history_length,
    robot_prediction_length=robot_prediction_length,
    speed_buffer_length=speed_buffer_length,
    use_generator=True  
)

obs, info = env.reset()

print("Observation space:", env.observation_space)
print("Shape of observation space:", env.observation_space.shape)
print("Action space:", env.action_space)

step = 0
total_reward = 0  # 累计奖励

while True:
    action = env.action_space.sample()

    # action = {
    #     'id': 1,
    #     'params0': [],
    #     'params1': [1e-3, 1e-3],
    # }

    obs, reward, terminated, truncated, info = env.step(action)
    step += 1
    total_reward += reward

    print("")
    print('action:', action)
    print('Reward:', reward)
    print('Info:', info)
    print('Step:', step)
    print(f'Current trajectory: {env.traj_index + 1}/{len(env.replay_traj_files)}')
    print(f'Total reward: {total_reward:.2f}')

    ## 目前我采取的计算总体reward的方式：由env.traj_index判断是否终止    （也可以移植到eval_env.py中的自定义函数中）
    if terminated:
        obs, info = env.reset()
        total_reward = 0  # 重置单条轨迹的奖励
        step = 0
        print('Trajectory ended. Resetting environment...')
        # break
        print('=====================================')
        print('')
        time.sleep(1)
        
env.close()
