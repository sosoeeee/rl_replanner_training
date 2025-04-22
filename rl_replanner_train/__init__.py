import gymnasium as gym
from rl_replanner_train.train_env import TrainEnv
from rl_replanner_train.eval_env import EvalEnv

gym.register(
    id='rl-replanner-train',
    entry_point='rl_replanner_train:SimulationWorld',
)