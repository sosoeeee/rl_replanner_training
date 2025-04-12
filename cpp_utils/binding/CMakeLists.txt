set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(cpp_utils main.cpp)

# Link the library to the python module (otherwise: undefined symbol: _ZTIN15nav2_costmap_2d9Costmap2DE)
target_link_libraries(cpp_utils PRIVATE map_loader)  
target_link_libraries(cpp_utils PRIVATE path_planner)
target_link_libraries(cpp_utils PRIVATE teb_local_planner)
target_link_libraries(cpp_utils PRIVATE map_voronoi)
target_link_libraries(cpp_utils PRIVATE traj_generator)

# Set the RPATH for the shared library (otherwise: libmap_loader.so: cannot open shared object file: No such file or directory)
set_target_properties(cpp_utils PROPERTIES
  INSTALL_RPATH "/usr/local/lib")

# install
install(TARGETS cpp_utils DESTINATION ./lib/python3.10/dist-packages)
