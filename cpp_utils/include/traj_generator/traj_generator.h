# ifndef _TRAJ_GENERATOR_H_
# define _TRAJ_GENERATOR_H_

#include <memory>

#include "map_loader/costmap_2d.hpp"
#include "map_voronoi/voronoigraph.h"

#include "teb_local_planner/pose_se2.h"
#include "teb_local_planner/optimal_planner.h"
#include "utils.h"

using namespace nav2_costmap_2d;
using namespace teb_local_planner;

struct Circle
{
    double x;
    double y;
    double radius;
};

class traj_generator
{
public:
    traj_generator(){};
    ~traj_generator(){};
    
    // initialize the static costmap and voronoi graph
    void initialize(const std::string &map_file, const std::string &planner_file, double path_resolution, double time_resolution);
    std::vector<Point> sampleTraj(Point start, Point end, double time_resolution);

private:
    void getInitPlan(std::vector<int> passby_nodes, const Point& start, const Point& end);
    void getNearestNode(Point p, int &node_id);
    void getCorridor();
    void getViaPoints();
    void getTrajectory();

    std::shared_ptr<Costmap2D> costmap_;
    VoronoiGraph voronoi_graph_;
    double path_resolution_; // resolution of the init path
    double time_resolution_; // resolution of the time

    // constraints
    std::vector<Circle> circles_;  // circles consitute the corridor
    ViaPointContainer via_points_; // via points
    
    // trajectory planning
    std::vector<PoseSE2> init_plan_; // trajectory points
    std::vector<Point> trajectory_; // trajectory points in 2D space sampled in time resolution

    // teb planner
    TebConfig cfg_;
    ObstContainer obstacles_;
};

# endif

