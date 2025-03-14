from setuptools import setup, find_packages

setup(name='gym-rl-replanner-training',
      version='1.0.0',
      packages=[package for package in find_packages()
                if package.startswith('gym_rl_replanner_train')],
      description='Gym-rl-replanner-training: A rl agent for path replanning, in order to predict human intention in human robot collaboration task. Only for training.',
      url='https://github.com/sosoeeee/rl_replanner_training.git',
      author='sosoeeee',
)