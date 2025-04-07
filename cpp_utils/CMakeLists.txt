cmake_minimum_required(VERSION 3.10.0)
project(cpp_utils VERSION 0.1.0 LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

# find_package
find_package(yaml_cpp_vendor REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)

# teb_local_planner
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(Boost REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)
if (Eigen3_FOUND)
  set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  message(STATUS "Found Eigen3: ${Eigen_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "Eigen3 not found. Please install Eigen3.")
endif()

set(TEB_EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(TEB_EXTERNAL_LIBS ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

# add_executable / add_library
set(map_library_name map_loader)

add_library(${map_library_name} SHARED
  src/${map_library_name}/map_mode.cpp
  src/${map_library_name}/map_io.cpp
  src/${map_library_name}/costmap_2d.cpp
  src/${map_library_name}/static_layer.cpp
  src/${map_library_name}/inflation_layer.cpp)

set(path_library_name path_planner)

add_library(${path_library_name} SHARED
  src/${path_library_name}/navfn.cpp
  src/${path_library_name}/navfn_planner_with_cone.cpp
  )

set(teb_local_planner_name teb_local_planner)

add_library(${teb_local_planner_name} SHARED
   src/${teb_local_planner_name}/timed_elastic_band.cpp
   src/${teb_local_planner_name}/optimal_planner.cpp
   src/${teb_local_planner_name}/obstacles.cpp
   src/${teb_local_planner_name}/teb_config.cpp
)

# include_directories / target_include_directories
target_include_directories(${map_library_name} PUBLIC ./include)
target_include_directories(${map_library_name} SYSTEM PRIVATE
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
target_include_directories(${map_library_name} SYSTEM PRIVATE
  ${yaml_cpp_vendor_INCLUDE_DIRS})

target_include_directories(${path_library_name} PUBLIC ./include)

target_include_directories(${teb_local_planner_name} PUBLIC ./include)
target_include_directories(${teb_local_planner_name} SYSTEM PUBLIC
  ${TEB_EXTERNAL_INCLUDE_DIRS})
target_include_directories(${teb_local_planner_name} SYSTEM PRIVATE
  ${yaml_cpp_vendor_INCLUDE_DIRS})
target_include_directories(${teb_local_planner_name} SYSTEM PRIVATE
  ${Boost_INCLUDE_DIRS})

# Linking and Dependencies
target_link_libraries(${map_library_name} 
  ${GRAPHICSMAGICKCPP_LIBRARIES})
target_link_libraries(${map_library_name}
  ${yaml_cpp_vendor_LIBRARIES})

target_link_libraries(${path_library_name} ${yaml_cpp_vendor_LIBRARIES})

target_link_libraries(${teb_local_planner_name} ${TEB_EXTERNAL_LIBS} ${yaml_cpp_vendor_LIBRARIES})

# install
install(TARGETS ${map_library_name} DESTINATION lib)
install(TARGETS ${path_library_name} DESTINATION lib)
install(TARGETS ${teb_local_planner_name} DESTINATION lib)
# install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

add_subdirectory(binding)