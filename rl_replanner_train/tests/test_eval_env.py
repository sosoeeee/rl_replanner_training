#!/usr/bin/python3
import sys
import os

# Add the workspace path to the PYTHONPATH
workspace_path = os.path.join(os.path.dirname(__file__) + "/../..")
sys.path.append(workspace_path)

from rl_replanner_train.eval_env import EvalEnv

print("Test Eval Environment")
print("=====================================")

# 设置环境参数
reward_weight = {
    'task': 1.0,
    'reg_angle': 0,
    'reg_depth': 0,
    'reg_replan': 0.05,
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
    map_setting_file='/home/rosdev/ros2_ws/rl_replanner_train/maps/tb3_classic/turtlebot3_world.yaml',
    planner_setting_file='/home/rosdev/ros2_ws/cpp_utils/include/path_planner/planner_setting.yaml',
    render_mode='ros',
    render_real_time_factor=4,
    obser_width=obser_width,
    replay_traj_path='/home/rosdev/ros2_ws/rl_replanner_train/data/eval_paths',
    human_history_length=human_history_length,
    robot_prediction_length=robot_prediction_length,
    speed_buffer_length=speed_buffer_length,
    use_generator=False  
)

obs, info = env.reset()

print("Observation space:", env.observation_space)
print("Shape of observation space:", env.observation_space.shape)
print("Action space:", env.action_space)

step = 0
total_reward = 0  # 累计奖励
episode_reward = 0  # 单条轨迹的奖励
traj_count = 0  # 已评估的轨迹数量

while True:
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward
    episode_reward += reward

    print('Observation:', obs)
    print('Reward:', reward)
    print(f'Current trajectory: {env.traj_index + 1}/{len(env.replay_traj_files)}')
    print(f'Episode reward: {episode_reward:.2f}')
    print(f'Total reward: {total_reward:.2f}')

    ## 目前我采取的计算总体reward的方式：由env.traj_index判断是否终止    （也可以移植到eval_env.py中的自定义函数中）
    if terminated:
        if env.traj_index >= len(env.replay_traj_files):
            print("All trajectories have been evaluated!")
            print(f'Final total reward: {total_reward:.2f}')
            break

        print(f"Trajectory {env.traj_index + 1} finished after {step + 1} timesteps")
        print(f'Episode reward: {episode_reward:.2f}')
        traj_count += 1
            
        obs, info = env.reset()
        episode_reward = 0  # 重置单条轨迹的奖励
    ##
    step += 1

env.close()
