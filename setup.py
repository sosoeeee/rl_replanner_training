from setuptools import setup, find_packages

setup(name='rl_replanner_train',
      version='1.0.0',
      packages=[package for package in find_packages()
                if package.startswith('rl_replanner_train')],
      description='rl_replanner_train A rl agent for path replanning, in order to predict human intention in human robot collaboration task. Only for training.',
      url='https://github.com/sosoeeee/rl_replanner_training.git',
      author='sosoeeee',
)