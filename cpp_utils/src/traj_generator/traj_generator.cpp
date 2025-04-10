#include "traj_generator/traj_generator.h"
#include "map_loader/map_io.hpp"

#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/optimal_planner.h"

using namespace nav2_costmap_2d;
using namespace teb_local_planner;

void traj_generator::initialize(const std::string &map_file, const std::string &planner_file, double path_resolution, double time_resolution)
{
    path_resolution_ = path_resolution;
    time_resolution_ = time_resolution;

    // load the costmap from yaml file
    unsigned int size_x, size_y;
    double resolution, origin_x, origin_y;
    int8_t *data;

    // read the yaml file
    LOAD_MAP_STATUS status = loadMapFromYaml(map_file, size_x, size_y, resolution, origin_x, origin_y, data);
    costmap_ = std::make_shared<Costmap2D>(size_x, size_y, resolution, origin_x, origin_y);

    // load static layer
    auto static_layer = StaticLayer(data, map_file);
    static_layer.onInitialize();
    static_layer.updateCosts(costmap_.get());
    
    // initialize the voronoi graph
    voronoi_graph_ = VoronoiGraph(costmap_);

    // initialize the teb planner
    cfg_.loadParamsFromYaml(planner_file);
    cfg_.checkParameters();
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

void traj_generator::getCorridor()
{
    // create circular corridor
    circles_.clear();
    double resolution = costmap_->getResolution();
    double dist;
    circle cur_circle;
    unsigned int mx, my;
    int map_x, map_y;

    // get the first circle
    PoseSE2 start_pose = init_plan_.front();
    cur_circle.x = start_pose.x();
    cur_circle.y = start_pose.y();
    costmap_->worldToMap(start_pose.x(), start_pose.y(), mx, my);
    map_x = static_cast<int>(mx);
    map_y = static_cast<int>(my);
    cur_circle.radius = static_cast<double>(voronoi_graph_.getDistance(map_x, map_y)) * resolution;
    circles_.push_back(cur_circle);

    // TODO: Bubble planner optimization --- batch sample
    for (const auto& pose : init_plan_) {
        // get the first pose outside the current circle
        dist = sqrt(pow(pose.x() - cur_circle.x, 2) + pow(pose.y() - cur_circle.y, 2));
        if (dist < cur_circle.radius) {
            continue;
        }

        // add a new circle to the corridor
        cur_circle.x = pose.x();
        cur_circle.y = pose.y();
        costmap_->worldToMap(pose.x(), pose.y(), mx, my);
        map_x = static_cast<int>(mx);
        map_y = static_cast<int>(my);
        cur_circle.radius = static_cast<double>(voronoi_graph_.getDistance(map_x, map_y)) * resolution;

        circles_.push_back(cur_circle);
    }

    // add the last circle to the corridor
    PoseSE2 end_pose = init_plan_.back();
    cur_circle.x = end_pose.x();
    cur_circle.y = end_pose.y();
    costmap_->worldToMap(end_pose.x(), end_pose.y(), mx, my);
    map_x = static_cast<int>(mx);
    map_y = static_cast<int>(my);
    cur_circle.radius = static_cast<double>(voronoi_graph_.getDistance(map_x, map_y)) * resolution;
    circles_.push_back(cur_circle);
}

void traj_generator::getViaPoints()
{
    // sample via points from the corridor
    via_points_.clear();

    std::random_device rd;
    std::mt19937 gen(rd());
    double px, py;
    double dist;

    // the ratio of the radius to the standard deviation
    // TODO: Make this a parameter, 3.0 means that 99.7% of the points are in the circle
    double sigma_factor = 3.0 

    for (const auto& circle : circles_) {
        // sample via points in the circle
        std::normal_distribution<double> dist_x(circle.x, circle.radius / sigma_factor);
        px = dist_x(gen);
        std::normal_distribution<double> dist_y(circle.y, circle.radius / sigma_factor);
        py = dist_y(gen);

        // check if the point is in the circle
        dist = sqrt(pow(px - circle.x, 2) + pow(py - circle.y, 2));
        while (dist >= circle.radius) {
            // resample the point
            px = dist_x(gen);
            py = dist_y(gen);
            dist = sqrt(pow(px - circle.x, 2) + pow(py - circle.y, 2));
        }
        via_points_.push_back(Eigen::Vector2d(px, py));
    }   
}

// get the trajectory from teb planner
void traj_generator::getTrajectory()
{
    trajectory_.clear();
    obstacles_.clear();

    // Setup circular corridor
    std::shared_ptr<CircularCorridor> corridor = std::make_shared<CircularCorridor>();
    corridor->clearCircles();  // Ensure corridor is empty
    for (const auto& circle : circles_) {
        corridor->addCircle(circle.x, circle.y, circle.radius);
    }
    obstacles_.push_back(corridor);

    // Initialize and run planner
    TebOptimalPlanner planner(cfg_, &obstacles_, &via_points_);
    if (!planner.plan(init_plan_)) {
        LOGGER_ERROR("teb_local_planner", "Failed to plan trajectory.");
        return;
    }

    // Get the planned trajectory
    std::vector<TrajectoryPointMsg> traj;
    planner.getFullTrajectory(traj);
    
    if (traj.empty()) {
        LOGGER_ERROR("teb_local_planner", "Planned trajectory is empty.");
        return;
    }

    // Pre-allocate memory for trajectory
    const size_t estimated_size = static_cast<size_t>(
        (traj.back().time_from_start - traj.front().time_from_start) / time_resolution_ + 1);
    trajectory_.reserve(estimated_size);

    // Add initial point
    trajectory_.push_back({traj[0].pose.x(), traj[0].pose.y()});
    
    double cur_time = traj[0].time_from_start;
    size_t idx = 0;
    const size_t traj_size = traj.size();

    // Resample trajectory at fixed time intervals
    while (idx < traj_size - 1) {
        cur_time += time_resolution_;

        // Find appropriate trajectory segment
        while (cur_time > traj[idx + 1].time_from_start) {
            idx++;
        }

        // Linear interpolation between trajectory points
        const double segment_time = traj[idx + 1].time_from_start - traj[idx].time_from_start;
        if (segment_time > 0) {
            const double alpha = (cur_time - traj[idx].time_from_start) / segment_time;
            const point p = {
                traj[idx].pose.x() + alpha * (traj[idx + 1].pose.x() - traj[idx].pose.x()),
                traj[idx].pose.y() + alpha * (traj[idx + 1].pose.y() - traj[idx].pose.y())
            };
            trajectory_.push_back(p);
        }
    }

    // Add final point if not already added
    if (std::abs(trajectory_.back().x - traj.back().pose.x()) > 1e-6 || std::abs(trajectory_.back().y - traj.back().pose.y()) > 1e-6) {
        trajectory_.push_back({traj.back().pose.x(), traj.back().pose.y()});
    }
}

std::vector<point> traj_generator::sampleTraj(point start, point end, double time_resolution)
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

    return trajectory_;
}