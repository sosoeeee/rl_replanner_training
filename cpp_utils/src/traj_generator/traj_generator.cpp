#include "traj_generator/traj_generator.h"
#include "map_loader/map_io.hpp"

void traj_generator::initialize(const std::string &yaml_file, double path_resolution, double time_resolution)
{
    path_resolution_ = path_resolution;
    time_resolution_ = time_resolution;

    // load the costmap from yaml file
    unsigned int size_x, size_y;
    double resolution, origin_x, origin_y;
    int8_t *data;

    // read the yaml file
    LOAD_MAP_STATUS status = loadMapFromYaml(yaml_file, size_x, size_y, resolution, origin_x, origin_y, data);
    costmap_ = std::make_shared<Costmap2D>(size_x, size_y, resolution, origin_x, origin_y);

    // load static layer
    auto static_layer = StaticLayer(data, yaml_file);
    static_layer.onInitialize();
    static_layer.updateCosts(costmap_.get());
    
    // initialize the voronoi graph
    voronoi_graph_ = VoronoiGraph(costmap_);
}

void traj_generator::getNearestNode(point p, int &node_id)
{
    // get the nearest voronoi node to the point p
    // TODO: Optimize the searching efficiency by using kd-tree or other methods
    std::vector<VoronoiNode> nodes = voronoi_graph_.getAllNodes();
    double min_dist = std::numeric_limits<double>::max();
    double wx, wy, dist;
    for (const auto& node : nodes) {
        costmap_->mapToWorld(node.getPosition().x, node.getPosition().y, wx, wy);
        dist = sqrt(pow(wx - p.x, 2) + pow(wy - p.y, 2));

        if (dist < min_dist) {
            min_dist = dist;
            node_id = node.getId();
        }
    }
}

void traj_generator::getInitPlan(std::vector<int> passby_nodes, const point& start, const point& end)
{
    init_plan_.clear();
    
    // 1. start to start node
    VoronoiNode start_node = voronoi_graph_.getNodeById(passby_nodes[0]);
    double start_wx, start_wy;
    costmap_->mapToWorld(start_node.getPosition().x, start_node.getPosition().y, start_wx, start_wy);
    // connect the start point to the start node according to path resolution
    double dist = sqrt(pow(start_wx - start.x, 2) + pow(start_wy - start.y, 2));
    int num_points = static_cast<int>(dist / path_resolution_);
    double dx = (start_wx - start.x) / num_points;
    double dy = (start_wy - start.y) / num_points;
    for (int i = 0; i < num_points; ++i) {
        init_plan_.push_back(PoseSE2(start.x + i * dx, start.y + i * dy, 0.0));
    }
    init_plan_.push_back(PoseSE2(start_wx, start_wy, 0.0));
    
    // 2. passby nodes
    for (int i = 0; i < passby_nodes.size() - 1; ++i) {
        VoronoiNode start_node = voronoi_graph_.getNodeById(passby_nodes[i]);
        std::vector<MapPoint> path_points = start_node.getPathById(passby_nodes[i + 1]);

        double last_wx, last_wy;
        double cur_wx, cur_wy;
        costmap_->mapToWorld(start_node.getPosition().x, start_node.getPosition().y, last_wx, last_wy);
        for (const auto& point : path_points) {
            costmap_->mapToWorld(point.x, point.y, cur_wx, cur_wy);
            dist = sqrt(pow(cur_wx - last_wx, 2) + pow(cur_wy - last_wy, 2));
            if (dist > path_resolution_) {
                init_plan_.push_back(PoseSE2(cur_wx, cur_wy, 0.0));
                last_wx = cur_wx;
                last_wy = cur_wy;
            }
            else
            {
                // if the distance is less than path resolution, we can skip this point
                continue;
            }
        }
    }

    // 3. end node to end
    VoronoiNode end_node = voronoi_graph_.getNodeById(passby_nodes.back());
    double end_wx, end_wy;
    costmap_->mapToWorld(end_node.getPosition().x, end_node.getPosition().y, end_wx, end_wy);
    // connect the end node to the end point according to path resolution
    dist = sqrt(pow(end_wx - end.x, 2) + pow(end_wy - end.y, 2));
    num_points = static_cast<int>(dist / path_resolution_);
    dx = (end.x - end_wx) / num_points;
    dy = (end.y - end_wy) / num_points;
    for (int i = 0; i < num_points; ++i) {
        init_plan_.push_back(PoseSE2(end_wx + i * dx, end_wy + i * dy, 0.0));
    }   
    init_plan_.push_back(PoseSE2(end.x, end.y, 0.0));
}

void traj_generator::sampleTraj(point start, point end, double time_resolution)
{
    // get the nearest voronoi node to the start point and end point
    int start_node_id, end_node_id;
    getNearestNode(start, start_node_id);
    getNearestNode(end, end_node_id);
    
    // sample the passby voronoi nodes from start node to end node
    std::vector<int> passby_nodes = voronoi_graph_.getPassbyNodes(start_node_id, end_node_id);

    // get initial path from voronoi graph
    getInitPlan(passby_nodes);

    // create circular corridor
    getCorridor();

    // sample via points from the corridor
    getViaPoints();

    // plan trajectory
    getTrajectory();
}   