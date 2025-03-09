#include "map_loader/map_io.hpp"
#include "map_loader/costmap_2d.hpp"
#include "map_loader/static_layer.hpp"
#include "map_loader/inflation_layer.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>

#include <iostream>

namespace py = pybind11;

using namespace nav2_map_server;
using namespace nav2_costmap_2d;

std::tuple<LOAD_MAP_STATUS, Costmap2D>
loadMap(const std::string &yaml_file) {
    // raw map data
    unsigned int size_x, size_y;
    double resolution, origin_x, origin_y;
    int8_t *data;

    // read the yaml file
    LOAD_MAP_STATUS status = loadMapFromYaml(yaml_file, size_x, size_y, resolution, origin_x, origin_y, data);
    std::unique_ptr<Costmap2D> map_ptr = std::make_unique<Costmap2D>(size_x, size_y, resolution, origin_x, origin_y);

    // load static layer
    auto static_layer = StaticLayer(data, yaml_file);
    static_layer.onInitialize();
    static_layer.updateCosts(map_ptr.get());

    // load inflation layer
    auto inflation_layer = InflationLayer(map_ptr.get(), yaml_file);
    inflation_layer.onInitialize();
    inflation_layer.updateCosts();

    // release the raw data
    delete[] data;

    return std::make_tuple(status, *map_ptr);
}

PYBIND11_MODULE(cpp_utils, m) {
    // Bind the enum
    py::enum_<LOAD_MAP_STATUS>(m, "LOAD_MAP_STATUS")
        .value("LOAD_MAP_SUCCESS", LOAD_MAP_SUCCESS)
        .value("MAP_DOES_NOT_EXIST", MAP_DOES_NOT_EXIST)
        .value("INVALID_MAP_METADATA", INVALID_MAP_METADATA)
        .value("INVALID_MAP_DATA", INVALID_MAP_DATA)
        .export_values();

    // Bind the Costmap2D class
    py::class_<Costmap2D, std::shared_ptr<Costmap2D>>(m, "Costmap2D_cpp")
        .def(py::init<unsigned int, unsigned int, double, double, double, unsigned char>())
        .def_property("size_x", &Costmap2D::getSizeInCellsX, nullptr)
        .def_property("size_y", &Costmap2D::getSizeInCellsY, nullptr)
        .def_property("resolution", &Costmap2D::getResolution, nullptr)
        .def_property("origin_x", &Costmap2D::getOriginX, nullptr)
        .def_property("origin_y", &Costmap2D::getOriginY, nullptr)
        .def_property("data", &Costmap2D::getCharMapToPy, nullptr)
        .def("getCost", static_cast<unsigned char (Costmap2D::*)(unsigned int, unsigned int) const>(&Costmap2D::getCost), "Get the cost of a cell in the costmap", py::arg("mx"), py::arg("my"));

    // Bind the loadMap function
    m.def("loadMap", &loadMap, "Load map from YAML into OccupancyGrid", py::arg("yaml_file"));
}

// Function to path plan
