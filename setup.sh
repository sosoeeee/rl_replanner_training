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

# build g2o with branch "20230806_git"
cd ./extern/g2o \
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

# temporary pkg for debugging
# pip install matplotlib 
# sudo apt-get install python3-tk

# RUN the following command to test the installation
# python3 ./rl_replanner_train/tests/test_env.py