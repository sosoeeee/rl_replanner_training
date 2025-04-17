import gymnasium as gym
from rl_replanner_train.train_env import TrainEnv

gym.register(
    id='rl-replanner-train',
    entry_point='rl_replanner_train:SimulationWorld',
)