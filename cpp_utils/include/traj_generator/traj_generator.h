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

class TrajGenerator
{
public:
    TrajGenerator(){};
    ~TrajGenerator(){};
    
    // initialize the static costmap and voronoi graph
    void initialize(const std::string &map_file, const std::string &planner_file, double path_resolution, double time_resolution);
    std::vector<Point> sampleTraj(Point start, Point end);

    // for visualization
    std::vector<PoseSE2> getInitPlan() const {return init_plan_;}
    std::vector<Circle> getCircles() const {return circles_;}
    std::vector<TrajectoryPointMsg> getRawTraj() const {return raw_trajectory_;}
    std::shared_ptr<Costmap2D> getCostmap() const {return costmap_;}
    std::vector<Point> getViaPoints() const {
        std::vector<Point> via_points;
        for (const auto& via_point : via_points_) {
            Point p = {
                via_point.x(),
                via_point.y()
            };
            via_points.push_back(p);
        }
        return via_points;
    }

private:
    void updateInitPlan(std::vector<int> passby_nodes, const Point& start, const Point& end);
    void getNearestNode(Point p, int &node_id);
    void updateCorridor();
    void updateViaPoints();
    void updateTrajectory();

    std::shared_ptr<Costmap2D> costmap_;
    std::unique_ptr<VoronoiGraph> voronoi_graph_;
    double path_resolution_; // resolution of the init path
    double time_resolution_; // resolution of the time

    // constraints
    std::vector<Circle> circles_;  // circles consitute the corridor
    ViaPointContainer via_points_; // via points
    
    // trajectory planning
    std::vector<PoseSE2> init_plan_; // trajectory points
    std::vector<TrajectoryPointMsg> raw_trajectory_;
    std::vector<Point> trajectory_; // trajectory points in 2D space sampled in time resolution

    // teb planner
    TebConfig cfg_;
    std::shared_ptr<ObstContainer> obstacles_;
};

# endif

