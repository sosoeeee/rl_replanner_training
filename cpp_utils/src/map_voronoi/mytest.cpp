// Testing Process:
// step1: cd build
// step2: cmake ..
// step3: make 
// step4: ./my_test

#include "map_loader/map_io.hpp"
#include "map_loader/costmap_2d.hpp"
#include "map_loader/static_layer.hpp"
#include "map_loader/inflation_layer.hpp"
#include "map_voronoi/voronoi.h"
#include "map_voronoi/voronoinode.h"
#include "map_voronoi/voronoigraph.h"
#include <iostream>
#include <fstream>
#include <string>
#include <tuple>

using namespace nav2_map_server;
using namespace nav2_costmap_2d;

std::tuple<LOAD_MAP_STATUS, std::shared_ptr<Costmap2D>>
loadMap(const std::string &yaml_file) {
    // raw map data
    unsigned int size_x, size_y;
    double resolution, origin_x, origin_y;
    int8_t *data;

    // read the yaml file
    LOAD_MAP_STATUS status = loadMapFromYaml(yaml_file, size_x, size_y, resolution, origin_x, origin_y, data);

    // Create a shared pointer to Costmap2D
    std::shared_ptr<Costmap2D> map_ptr = std::make_shared<Costmap2D>(size_x, size_y, resolution, origin_x, origin_y);

    // load static layer
    auto static_layer = StaticLayer(data, yaml_file);
    static_layer.onInitialize();
    static_layer.updateCosts(map_ptr.get());

    // // load inflation layer
    // auto inflation_layer = InflationLayer(map_ptr.get(), yaml_file);
    // inflation_layer.onInitialize();
    // inflation_layer.updateCosts();

    // release the raw data
    delete[] data;

    return std::make_tuple(status, map_ptr);
}

int main() {
    // Load the map from the YAML file
    auto [status, costmap] = loadMap("/home/rosdev/ros2_ws/rl_replanner_train/maps/tb3_classic/turtlebot3_world.yaml");

    // Get the map dimensions
    unsigned int sizeX = costmap->getSizeInCellsX();
    unsigned int sizeY = costmap->getSizeInCellsY();

    std::cout << "Map loaded (" << sizeX << "x" << sizeY << ").\n";

    // // Create and visualize Voronoi graph
    VoronoiGraph voronoigraph(costmap);
    voronoigraph.getVoronoiGraph();
    voronoigraph.visualizeVoronoi("initial.ppm");
    std::cout << "Generated initial frame.\n";
    voronoigraph.findAllPaths(0,3);

    return 0;
}