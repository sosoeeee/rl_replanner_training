#include "traj_generator/traj_generator.h"
#include "map_loader/map_io.hpp"
#include "map_loader/static_layer.hpp"

#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/optimal_planner.h"

using namespace nav2_costmap_2d;
using namespace teb_local_planner;
using namespace nav2_map_server;

void TrajGenerator::initialize(const std::string &map_file, const std::string &planner_file, double path_resolution, double time_resolution)
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

    // get robot radius
    YAML::Node doc = YAML::LoadFile(expand_user_home_dir_if_needed(map_file, get_home_dir()));

    YAML::Node inflation_layer_node = doc["inflation_layer"];
    if (!inflation_layer_node) {
        throw std::runtime_error("Failed to find 'inflation_layer' node in YAML file");
    }
    robot_radius_ = yaml_get_value<double>(inflation_layer_node, "robot_radius");
    
    // initialize the voronoi graph
    voronoi_graph_ = std::make_unique<VoronoiGraph>(costmap_);
    voronoi_graph_->getVoronoiGraph();

    // initialize the teb planner
    cfg_.loadParamsFromYaml(planner_file);
    cfg_.checkParameters();

    // obstacle container
    obstacles_ = std::make_shared<ObstContainer>();

    // initialize the teb planner
    // planner_ = std::make_unique<TebOptimalPlanner>(cfg_, obstacles_.get(), &via_points_);
}

void TrajGenerator::getNearestNode(Point p, int &node_id)
{
    // get the nearest voronoi node to the Point p
    // TODO: Optimize the searching efficiency by using kd-tree or other methods
    std::vector<VoronoiNode> nodes = voronoi_graph_->getAllNodes();
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

void TrajGenerator::updateInitPlan(std::vector<int> passby_nodes, const Point& start, const Point& end)
{
    init_plan_.clear();
    
    // 1. start to start node
    VoronoiNode start_node = voronoi_graph_->getNodeById(passby_nodes[0]);
    double start_wx, start_wy;
    costmap_->mapToWorld(start_node.getPosition().x, start_node.getPosition().y, start_wx, start_wy);
    // connect the start Point to the start node according to path resolution
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
        VoronoiNode start_node = voronoi_graph_->getNodeById(passby_nodes[i]);
        std::vector<MapPoint> path_points = start_node.getPathById(passby_nodes[i + 1]);

        double last_wx, last_wy;
        double cur_wx, cur_wy;
        costmap_->mapToWorld(start_node.getPosition().x, start_node.getPosition().y, last_wx, last_wy);
        for (const auto& Point : path_points) {
            costmap_->mapToWorld(Point.x, Point.y, cur_wx, cur_wy);
            dist = sqrt(pow(cur_wx - last_wx, 2) + pow(cur_wy - last_wy, 2));

            // TODO: When path resolution is smaller then the distance between two points, actual path resolution is the distance between two points
            // Because the point distance is the map resolution, so there is no need to add the middle "point" to satisfy the path resolution
            if (dist > path_resolution_) {
                init_plan_.push_back(PoseSE2(cur_wx, cur_wy, 0.0));
                last_wx = cur_wx;
                last_wy = cur_wy;
            }
            else
            {
                // if the distance is less than path resolution, we can skip this Point
                continue;
            }
        }
    }

    // 3. end node to end
    VoronoiNode end_node = voronoi_graph_->getNodeById(passby_nodes.back());
    double end_wx, end_wy;
    costmap_->mapToWorld(end_node.getPosition().x, end_node.getPosition().y, end_wx, end_wy);
    // connect the end node to the end Point according to path resolution
    dist = sqrt(pow(end_wx - end.x, 2) + pow(end_wy - end.y, 2));
    num_points = static_cast<int>(dist / path_resolution_);
    dx = (end.x - end_wx) / num_points;
    dy = (end.y - end_wy) / num_points;

    // skip the end_node point, i starts from 1
    for (int i = 1; i < num_points; ++i) {
        init_plan_.push_back(PoseSE2(end_wx + i * dx, end_wy + i * dy, 0.0));
    }   
    init_plan_.push_back(PoseSE2(end.x, end.y, 0.0));
}

void TrajGenerator::updateCorridor()
{
    // create circular corridor
    circles_.clear();
    double resolution = costmap_->getResolution();
    double dist;
    Circle cur_circle;
    unsigned int mx, my;
    int map_x, map_y;

    // get the first circle
    PoseSE2 start_pose = init_plan_.front();
    cur_circle.x = start_pose.x();
    cur_circle.y = start_pose.y();
    costmap_->worldToMap(start_pose.x(), start_pose.y(), mx, my);
    map_x = static_cast<int>(mx);
    map_y = static_cast<int>(my);
    cur_circle.radius = static_cast<double>(voronoi_graph_->getDistance(map_x, map_y)) * resolution - robot_radius_;
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
        cur_circle.radius = static_cast<double>(voronoi_graph_->getDistance(map_x, map_y)) * resolution - robot_radius_;

        circles_.push_back(cur_circle);
    }

    // add the last circle to the corridor
    PoseSE2 end_pose = init_plan_.back();
    cur_circle.x = end_pose.x();
    cur_circle.y = end_pose.y();
    costmap_->worldToMap(end_pose.x(), end_pose.y(), mx, my);
    map_x = static_cast<int>(mx);
    map_y = static_cast<int>(my);
    cur_circle.radius = static_cast<double>(voronoi_graph_->getDistance(map_x, map_y)) * resolution - robot_radius_;
    circles_.push_back(cur_circle);
}

void TrajGenerator::updateViaPoints()
{
    // sample via points from the corridor
    via_points_.clear();

    std::random_device rd;
    std::mt19937 gen(rd());
    double px, py;
    double dist;

    // the ratio of the radius to the standard deviation
    // TODO: Make this a parameter, 3.0 means that 99.7% of the points are in the circle
    double sigma_factor = 3.0;

    // Skip first and last circles, start from index 1 and end before the last circle
    for (size_t i = 1; i < circles_.size() - 1; ++i) {
        const auto& circle = circles_[i];
        // sample via points in the circle
        std::normal_distribution<double> dist_x(circle.x, circle.radius / sigma_factor);
        px = dist_x(gen);
        std::normal_distribution<double> dist_y(circle.y, circle.radius / sigma_factor);
        py = dist_y(gen);

        // check if the Point is in the circle
        dist = sqrt(pow(px - circle.x, 2) + pow(py - circle.y, 2));
        while (dist >= circle.radius) {
            // resample the Point
            px = dist_x(gen);
            py = dist_y(gen);
            dist = sqrt(pow(px - circle.x, 2) + pow(py - circle.y, 2));
        }
        via_points_.push_back(Eigen::Vector2d(px, py));
    }   

    // LOGGER_INFO("teb_local_planner", "After sampling, number of via points: %d", via_points_.size());
}

// get the trajectory from teb planner
void TrajGenerator::updateTrajectory()
{
    trajectory_.clear();
    obstacles_->clear();

    /* ============== the viapoints constrain has been able to constrain the path into its origin homotopy class ============== */
    // Setup circular corridor
    // std::shared_ptr<CircularCorridor> corridor = std::make_shared<CircularCorridor>();
    // corridor->clearCircles();  // Ensure corridor is empty
    // for (const auto& circle : circles_) {
    //     corridor->addCircle(circle.x, circle.y, circle.radius);
    // }
    // obstacles_->push_back(corridor);
    /* ========================================== release this constrain to speed up ========================================== */
    
    // planner_->setObstVector(obstacles_.get());
    // planner_->setViaPoints(&via_points_);
    // Initialize and run planner
    auto start_time = std::chrono::high_resolution_clock::now();
    TebOptimalPlanner planner(cfg_, obstacles_.get(), &via_points_);

    // use via_points_ to create init plan
    std::vector<PoseSE2> init_plan_via_points;
    init_plan_via_points.push_back(init_plan_.front());
    for (const auto& via_point : via_points_) {
        init_plan_via_points.push_back(PoseSE2(via_point.x(), via_point.y(), 0.0));
    }
    init_plan_via_points.push_back(init_plan_.back());

    if (!planner.plan(init_plan_via_points)) {
        LOGGER_ERROR("teb_local_planner", "Failed to plan trajectory.");
        return;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;
    LOGGER_INFO("teb_local_planner", "Planning time: %f ms", elapsed_time.count() * 1000);

    // Get the planned trajectory
    planner.getFullTrajectory(raw_trajectory_);
    
    if (raw_trajectory_.empty()) {
        LOGGER_ERROR("teb_local_planner", "Planned trajectory is empty.");
        return;
    }

    // Pre-allocate memory for trajectory
    const size_t estimated_size = static_cast<size_t>(
        (raw_trajectory_.back().time_from_start - raw_trajectory_.front().time_from_start) / time_resolution_ + 1);
    trajectory_.reserve(estimated_size);

    // Add initial Point
    trajectory_.push_back({raw_trajectory_[0].pose.x(), raw_trajectory_[0].pose.y()});

    // LOGGER_INFO("teb_local_planner", "trajectory size: %zu", trajectory_.size());
    // LOGGER_INFO("teb_local_planner", "Initial trajectory point: (%f, %f)", trajectory_.back().x, trajectory_.back().y);
    
    double cur_time = raw_trajectory_[0].time_from_start;
    double total_time = raw_trajectory_.back().time_from_start;
    size_t idx = 0;
    const size_t traj_size = raw_trajectory_.size();

    // Resample trajectory at fixed time intervals
    while (idx < traj_size - 1) {
        cur_time += time_resolution_;
        if (cur_time > total_time) {
            break;
        }

        // Find appropriate trajectory segment
        while (cur_time > raw_trajectory_[idx + 1].time_from_start) {
            idx++;
        }

        // Linear interpolation between trajectory points
        const double segment_time = raw_trajectory_[idx + 1].time_from_start - raw_trajectory_[idx].time_from_start;
        if (segment_time > 0) {
            const double alpha = (cur_time - raw_trajectory_[idx].time_from_start) / segment_time;
            const Point p = {
                raw_trajectory_[idx].pose.x() + alpha * (raw_trajectory_[idx + 1].pose.x() - raw_trajectory_[idx].pose.x()),
                raw_trajectory_[idx].pose.y() + alpha * (raw_trajectory_[idx + 1].pose.y() - raw_trajectory_[idx].pose.y())
            };
            trajectory_.push_back(p);
            
            // LOGGER_INFO("teb_local_planner", "");
            // LOGGER_INFO("teb_local_planner", "trajectory size: %zu", trajectory_.size());
            // LOGGER_INFO("teb_local_planner", "Interpolated trajectory point: (%f, %f)", p.x, p.y);
            // LOGGER_INFO("teb_local_planner", "Current time: %f", cur_time);
            // LOGGER_INFO("teb_local_planner", "RAW Trajectory size: %zu, idx: %zu", raw_trajectory_.size(), idx);
            // LOGGER_INFO("teb_local_planner", "Alpha: %f, from %f to %f", alpha, raw_trajectory_[idx].time_from_start, raw_trajectory_[idx + 1].time_from_start);
        }
    }

    // Add final Point if not already added
    if (std::abs(trajectory_.back().x - raw_trajectory_.back().pose.x()) > 1e-6 || std::abs(trajectory_.back().y - raw_trajectory_.back().pose.y()) > 1e-6) {
        trajectory_.push_back({raw_trajectory_.back().pose.x(), raw_trajectory_.back().pose.y()});
    }

    // LOGGER_INFO("teb_local_planner", "Final trajectory point: (%f, %f)", trajectory_.back().x, trajectory_.back().y);
    // LOGGER_INFO("teb_local_planner", "Total trajectory size: %zu", trajectory_.size());
}

std::vector<Point> TrajGenerator::sampleTraj(Point start, Point end)
{
    // get the nearest voronoi node to the start Point and end Point
    int start_node_id, end_node_id;
    getNearestNode(start, start_node_id);
    getNearestNode(end, end_node_id);

    // TODO: Update the voronoi graph with "Bubble technique"

    // // debug
    // LOGGER_INFO("teb_local_planner", "Start node ID: %d, End node ID: %d", start_node_id, end_node_id);
    
    // sample the passby voronoi nodes from start node to end node
    std::vector<int> passby_nodes = voronoi_graph_->getPassbyNodes(start_node_id, end_node_id);

    // // debug
    // LOGGER_INFO("teb_local_planner", "Passby nodes: ");
    // for (const auto& node_id : passby_nodes) {
    //     LOGGER_INFO("teb_local_planner", "%d", node_id);
    // }

    // get initial path from voronoi graph
    updateInitPlan(passby_nodes, start, end);

    // LOGGER_INFO("teb_local_planner", "Initial path: ");
    // int idx = 0;
    // for (const auto& pose : init_plan_) {
    //     LOGGER_INFO("teb_local_planner", "Pose %d: (%f, %f)", idx++, pose.x(), pose.y());
    // }

    // create circular corridor
    updateCorridor();

    // sample via points from the corridor
    updateViaPoints();

    // plan trajectory
    updateTrajectory();

    return trajectory_;
}

std::vector<Point> TrajGenerator::sampleDistinctHomotopyTrajs(Point start, Point end)
{   
    bool resample = false;
    if (!last_start_point_ || !last_end_point_)
    {
        last_start_point_ = std::make_unique<Point>(start);
        last_end_point_ = std::make_unique<Point>(end);
        resample = true;
    }
    else if (std::abs(last_start_point_->x - start.x) > 1e-6 || std::abs(last_start_point_->y - start.y) > 1e-6 ||
             std::abs(last_end_point_->x - end.x) > 1e-6 || std::abs(last_end_point_->y - end.y) > 1e-6)
    {
        last_start_point_ = std::make_unique<Point>(start);
        last_end_point_ = std::make_unique<Point>(end);
        resample = true;
        LOGGER_INFO("teb_local_planner", "Start point or end point changed, resampling trajectories from distinct homotopies.");
    }

    if (resample)
    {
        all_passby_nodes_.clear();
        // get the nearest voronoi node to the start Point and end Point
        int start_node_id, end_node_id;
        getNearestNode(start, start_node_id);
        getNearestNode(end, end_node_id);

        std::vector<std::vector<Point>> trajectories;
        all_passby_nodes_ = voronoi_graph_->findAllPaths(start_node_id, end_node_id);
    }

    if (sample_count_ >= all_passby_nodes_.size())
    {
        LOGGER_INFO("teb_local_planner", "All distinct homotopy trajectories have been sampled.");
        sample_count_ = 0;
        return {};
    }

    std::vector<int> passby_nodes = all_passby_nodes_[sample_count_];
    sample_count_++;

    // get initial path from voronoi graph
    // auto start_time = std::chrono::high_resolution_clock::now();

    updateInitPlan(passby_nodes, start, end);
    // auto init_plan_time = std::chrono::high_resolution_clock::now();
    // auto init_plan_duration = std::chrono::duration_cast<std::chrono::milliseconds>(init_plan_time - start_time);
    // LOGGER_INFO("teb_local_planner", "Init plan took %ld ms", init_plan_duration.count());

    // create circular corridor
    updateCorridor();
    // auto corridor_time = std::chrono::high_resolution_clock::now();
    // auto corridor_duration = std::chrono::duration_cast<std::chrono::milliseconds>(corridor_time - init_plan_time);
    // LOGGER_INFO("teb_local_planner", "Corridor creation took %ld ms", corridor_duration.count());

    // sample via points from the corridor
    updateViaPoints();
    // auto via_points_time = std::chrono::high_resolution_clock::now();
    // auto via_points_duration = std::chrono::duration_cast<std::chrono::milliseconds>(via_points_time - corridor_time);
    // LOGGER_INFO("teb_local_planner", "Via points sampling took %ld ms", via_points_duration.count());

    // plan trajectory
    updateTrajectory();
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto traj_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - via_points_time);
    // auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // LOGGER_INFO("teb_local_planner", "Trajectory optimization took %ld ms", traj_duration.count());
    // LOGGER_INFO("teb_local_planner", "Total planning took %ld ms", total_duration.count());

    return trajectory_;
}