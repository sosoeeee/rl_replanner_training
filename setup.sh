#!/bin/bash

# install python packages
pip install gymnasium==0.29.1
pip install billiard
pip install -e .

# build pybind11
cd ./extern/pybind11 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && sudo make install

# build c++ library
cd ./cpp_utils \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install

# build teb_planner
# sudo apt update && apt upgrade -y
# rosdep update                                                # set up network proxy first
# rosdep install -y --ignore-src --from-paths ./extern/teb_planner/src -r
# cd ./extern/teb_planner/
# colcon build --symlink-install

# RUN the following command to test the installation
# python3 ./rl_replanner_train/tests/test_env.py