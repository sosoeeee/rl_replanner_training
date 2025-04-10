# ifndef _TRAJ_GENERATOR_H_
# define _TRAJ_GENERATOR_H_

#include <memory>

#include "map_loader/costmap_2d.hpp"
#include "map_voronoi/voronoigraph.h"
#include "teb_local_planner/pose_se2.h"

using namespace nav2_costmap_2d;
using namespace teb_local_planner;

struct point
{
    double x;
    double y;
};

struct circle
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
    void initialize(const std::string &yaml_file, double path_resolution, double time_resolution);
    void sampleTraj(point start, point end, double time_resolution);

private:
    void getInitPlan(std::vector<int> passby_nodes, const point& start, const point& end);
    void getNearestNode(point p, int &node_id);
    void getCorridor();
    void getViaPoints();
    void getTrajectory();

    std::shared_ptr<Costmap2D> costmap_;
    VoronoiGraph voronoi_graph_;
    double path_resolution_; // resolution of the path
    double time_resolution_; // resolution of the time

    // constraints
    std::vector<circle> circles_;  // circles consitute the corridor
    std::vector<point> via_points_; // via points
    
    // trajectory planning
    std::vector<PoseSE2> init_plan_; // trajectory points
    std::vector<point> trajectory_; // trajectory points in 2D space sampled in time resolution
};

# endif

