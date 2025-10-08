// Testing Process:
// step1: cd build
// step2: cmake ..
// step3: make 
// step4: ./my_test
// sudo make install

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
#include <vector>
#include <numeric>
#include <limits>
#include <sstream>

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

// 用于存储从文件中读取的轨迹点
struct Point {
    double x;
    double y;
};

// 从txt文件加载轨迹数据
std::vector<Point> loadTrajectoryFromFile(const std::string& filename) {
    std::vector<Point> trajectory;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open trajectory file: " << filename << std::endl;
        return trajectory;
    }

    std::string line;
    while (std::getline(file, line)) {
        // 忽略空行或以'#'开头的注释行
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        Point p;
        // 从行中解析 x 和 y
        if (ss >> p.x >> p.y) {
            trajectory.push_back(p);
        }
    }

    std::cout << "Loaded " << trajectory.size() << " points from " << filename << std::endl;
    return trajectory;
}


int main(int argc, char* argv[]) {
    // 1. 加载地图
    // 您可以根据需要更改地图文件路径
    auto [status, costmap] = loadMap("/home/rosdev/ros2_ws/train_env/rl_replanner_training/rl_replanner_train/maps/real_maps/phy2.yaml");
    if (status != LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
        std::cerr << "Failed to load map." << std::endl;
        return -1;
    }
    std::cout << "Map loaded (" << costmap->getSizeInCellsX() << "x" << costmap->getSizeInCellsY() << ").\n";

    // 2. 初始化Voronoi图，用于距离计算
    VoronoiGraph voronoigraph(costmap);
    std::cout << "Voronoi graph initialized for distance calculations.\n";

    // 3. 加载轨迹文件
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_trajectory_file>" << std::endl;
        return -1;
    }
    std::string trajectory_path = argv[1];
    std::cout << "Analyzing trajectory: " << trajectory_path << std::endl;

    std::vector<Point> trajectory = loadTrajectoryFromFile(trajectory_path);
    if (trajectory.empty()) {
        return -1;
    }

    // 4. 计算每个轨迹点到最近障碍物的距离并求和
    double total_distance = 0.0;
    int valid_points = 0;

    for (const auto& point : trajectory) {
        unsigned int map_x, map_y;
        // 将世界坐标（米）转换为地图坐标（栅格）
        if (costmap->worldToMap(point.x, point.y, map_x, map_y)) {
            // 调用 getDistance 获取距离（单位：栅格）
            float dist = voronoigraph.getDistance(map_x, map_y);
            total_distance += dist;
            valid_points++;
        } else {
            std::cout << "Warning: Point (" << point.x << ", " << point.y << ") is outside the map bounds." << std::endl;
        }
    }

    // 5. 计算并打印平均距离
    if (valid_points > 0) {
        double avg_distance_cells = total_distance / valid_points;
        double avg_distance_meters = avg_distance_cells * costmap->getResolution();
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Safety Analysis Results for: " << trajectory_path << std::endl;
        std::cout << "Average distance to nearest obstacle: " << std::endl;
        std::cout << "  - In cells: " << avg_distance_cells << std::endl;
        std::cout << "  - In meters: " << avg_distance_meters << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    } else {
        std::cerr << "Error: No valid trajectory points found within the map." << std::endl;
    }

    return 0;
}