#!/bin/bash

# install python packages
pip install gymnasium==0.29.1
pip install billiard

# build pybind11 with branch "stable"
cd ./extern/pybind11 || exit 
sudo rm -rf build/ 
mkdir -p build 
cd build || exit  
cmake .. 
sudo make install

# cd to the root directory
cd ../../../ || exit

# build g2o with branch "2020410_git"
# install prerequisites
sudo apt update -q
sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev
cd ./extern/g2o || exit  
sudo rm -rf build
mkdir -p build 
cd build || exit  
cmake .. 
sudo make install # run it twice to install the library and the headers. Otherwise, the lib will not be found!

# cd to the root directory
cd ../../../ || exit

# build c++ library
cd ./cpp_utils || exit 
sudo rm -rf build/ 
mkdir -p build 
cd build || exit  
cmake .. 
make 
sudo make install

# cd to the root directory
cd ../../ || exit

pip install -e .

# temporary pkg for debugging
# pip install matplotlib 
# sudo apt-get install python3-tk

# RUN the following command to test the installation
# python3 ./rl_replanner_train/tests/test_env.py