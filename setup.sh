#!/bin/bash

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