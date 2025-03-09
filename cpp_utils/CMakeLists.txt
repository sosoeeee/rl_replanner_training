cmake_minimum_required(VERSION 3.10.0)
project(cpp_utils VERSION 0.1.0 LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

# find_package
find_package(yaml_cpp_vendor REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)

# add_executable / add_library
set(map_library_name map_loader)

add_library(${map_library_name} SHARED
  src/map_mode.cpp
  src/map_io.cpp
  src/costmap_2d.cpp
  src/static_layer.cpp
  src/inflation_layer.cpp)

# include_directories / target_include_directories
target_include_directories(${map_library_name} PUBLIC ./include)
target_include_directories(${map_library_name} SYSTEM PRIVATE
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
target_include_directories(${map_library_name} SYSTEM PRIVATE
  ${yaml_cpp_vendor_INCLUDE_DIRS})

# Linking and Dependencies
target_link_libraries(${map_library_name} 
  ${GRAPHICSMAGICKCPP_LIBRARIES})
target_link_libraries(${map_library_name}
  ${yaml_cpp_vendor_LIBRARIES})

# install
install(TARGETS ${map_library_name} DESTINATION lib)
# install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

add_subdirectory(binding)