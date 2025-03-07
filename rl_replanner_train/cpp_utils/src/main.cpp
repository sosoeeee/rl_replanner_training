#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>

namespace py = pybind11;

#include <map_io.hpp>
#include <costmap_2d.hpp>

std::tuple<LOAD_MAP_STATUS, nav2_costmap_2d::Costmap2D>
loadMap(const std::string &yaml_file) {
   nav2_costmap_2d::Costmap2D map;
   // Fill in the map...
   LOAD_MAP_STATUS status = SUCCESS; // or FAILURE based on your logic
   return std::make_tuple(status, map);
}

PYBIND11_MODULE(example, m) {
    // Bind the enum
    py::enum_<LOAD_MAP_STATUS>(m, "LOAD_MAP_STATUS")
        .value("LOAD_MAP_SUCCESS", LOAD_MAP_SUCCESS)
        .value("MAP_DOES_NOT_EXIST", MAP_DOES_NOT_EXIST)
        .value("INVALID_MAP_METADATA", INVALID_MAP_METADATA)
        .value("INVALID_MAP_DATA", INVALID_MAP_DATA)
        .export_values();

    // Bind the Costmap2D class
    py::class_<nav2_costmap_2d::Costmap2D, std::shared_ptr<nav2_costmap_2d::Costmap2D>>(m, "Costmap2D_cpp")
        // Bind necessary constructors and member functions here.
        ;

    // Bind the loadMap function
    m.def("loadMap", &loadMap, "Load map from YAML into OccupancyGrid")
}

// Function to path plan
