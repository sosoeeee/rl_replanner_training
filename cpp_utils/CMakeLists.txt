cmake_minimum_required(VERSION 3.10.0)
project(pybindtest VERSION 0.1.0 LANGUAGES C CXX)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(SumFunction src/main.cpp)
install(TARGETS SumFunction DESTINATION ./lib/python3.10/dist-packages)
