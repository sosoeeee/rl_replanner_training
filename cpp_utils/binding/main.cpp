#include "map_loader/map_io.hpp"
#include "map_loader/costmap_2d.hpp"
#include "map_loader/static_layer.hpp"
#include "map_loader/inflation_layer.hpp"

#include "path_planner/navfn_planner_with_cone.hpp"

#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/optimal_planner.h"

#include "traj_generator/traj_generator.h"

#include "utils.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tuple>

#include <iostream>

namespace py = pybind11;

using namespace nav2_map_server;
using namespace nav2_costmap_2d;
using namespace nav2_navfn_planner_with_cone;
using namespace teb_local_planner;

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
        .def("getCost", static_cast<unsigned char (Costmap2D::*)(unsigned int, unsigned int) const>(&Costmap2D::getCost), "Get the cost of a cell in the costmap", py::arg("mx"), py::arg("my"))
        .def("getCostByIndex", static_cast<unsigned char (Costmap2D::*)(unsigned int) const>(&Costmap2D::getCost), "Get the cost of a cell in the costmap by index", py::arg("index"))
        .def("getPartialCostmap", &Costmap2D::getPartialCostmap, "Get a partial costmap", py::arg("wx"), py::arg("wy"), py::arg("wx_size"), py::arg("wy_size"));
    
    // Bind the point data structure
    py::class_<Point>(m, "Point")
        .def(py::init<float, float>())
        .def_readwrite("x", &Point::x)
        .def_readwrite("y", &Point::y);

    py::class_<Circle>(m, "Circle")
        .def(py::init<float, float, float>())
        .def_readwrite("x", &Circle::x)
        .def_readwrite("y", &Circle::y)
        .def_readwrite("radius", &Circle::radius);

    py::class_<NavfnPlannerWithCone, std::shared_ptr<NavfnPlannerWithCone>>(m, "PathPlanner")
        .def(py::init())
        .def_property("inflated_distance", &NavfnPlannerWithCone::getInflatedDistance, nullptr)
        .def("configure", &NavfnPlannerWithCone::configure, 
            "Configure the path planner based on the yaml file", py::arg("costmap"), py::arg("yaml_filename"))
        .def("plan", &NavfnPlannerWithCone::createPlan, 
            "Create a plan from start and goal poses", py::arg("start"), py::arg("goal"))
        .def("loadCone", &NavfnPlannerWithCone::loadCone, 
            "Load the cone into the map", py::arg("cone_center"), py::arg("current_pos"), py::arg("radius"), py::arg("is_enabled"));

    // Bind the loadMap function
    m.def("loadMap", &loadMap, "Load map from YAML into OccupancyGrid", py::arg("yaml_file"));

    // Bind the msg
    py::class_<PoseSE2, std::shared_ptr<PoseSE2>>(m, "PoseSE2")
        .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("theta"))
        .def_property("x", static_cast<double& (PoseSE2::*)()>(&PoseSE2::x), nullptr)
        .def_property("y", static_cast<double& (PoseSE2::*)()>(&PoseSE2::y), nullptr)
        .def_property("theta", static_cast<double& (PoseSE2::*)()>(&PoseSE2::theta), nullptr);

    py::class_<VelSE2>(m, "VelSE2")
        .def(py::init<double, double, double>(), py::arg("vx"), py::arg("vy"), py::arg("omega"))
        .def_readwrite("vx", &VelSE2::vx)
        .def_readwrite("vy", &VelSE2::vy)
        .def_readwrite("omega", &VelSE2::omega);

    py::class_<TrajectoryPointMsg>(m, "TrajectoryPointMsg")
        .def(py::init())
        .def_readwrite("pose", &TrajectoryPointMsg::pose)
        .def_readwrite("velocity", &TrajectoryPointMsg::velocity)
        .def_readwrite("acceleration", &TrajectoryPointMsg::acceleration)
        .def_readwrite("time_from_start", &TrajectoryPointMsg::time_from_start);

    // Bind the obstacle class
    py::class_<Obstacle, std::shared_ptr<Obstacle>>(m, "Obstacle");
    py::class_<CircularObstacle, Obstacle, std::shared_ptr<CircularObstacle>>(m, "CircularObstacle")
        .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("radius"));
    py::class_<PointObstacle, Obstacle, std::shared_ptr<PointObstacle>>(m, "PointObstacle")
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"));
    py::class_<CircularCorridor, Obstacle, std::shared_ptr<CircularCorridor>>(m, "CircularCorridor")
        .def(py::init())
        .def("addCircle", static_cast<void (CircularCorridor::*)(double, double, double)>(&CircularCorridor::addCircle), 
            "Extend the corridor", py::arg("x"), py::arg("y"), py::arg("radius"))
        .def("clear", &CircularCorridor::clearCircles, 
            "Clear all circles from the corridor");

    // Bind the Trajectory generator class
    py::class_<VoronoiGraph, std::shared_ptr<VoronoiGraph>>(m, "VoronoiGraph")
        .def("getAllNodes",
             static_cast<std::vector<VoronoiNode>& (VoronoiGraph::*)()>(&VoronoiGraph::getAllNodes),
             py::return_value_policy::reference_internal,
             "Get all nodes in the Voronoi graph")
        .def("getNodeById",
             static_cast<VoronoiNode& (VoronoiGraph::*)(int)>(&VoronoiGraph::getNodeById),
             py::return_value_policy::reference_internal,
             "Get a node by its ID", py::arg("id"))
        .def("getPassbyNodes", &VoronoiGraph::getPassbyNodes,
            "Get passby nodes between two nodes", py::arg("start_id"), py::arg("end_id"))
        .def("findAllPaths", &VoronoiGraph::findAllPaths,
            "Find all paths between two nodes", py::arg("start_id"), py::arg("end_id"))
        .def("getDistance", &VoronoiGraph::getDistance,
            "Get distance to nearest obstacle", py::arg("x"), py::arg("y"))
        .def("visualizeVoronoi", &VoronoiGraph::visualizeVoronoi,
            "Visualize the original Voronoi graph", py::arg("filename"))
        .def("visualizeVoronoiModified", &VoronoiGraph::visualizeVoronoiModified,
            "Visualize the modified Voronoi graph", py::arg("filename"))
        .def("updateStartNeighbor", static_cast<void (VoronoiGraph::*)()>(&VoronoiGraph::getStartNeighbor),
            "Update unique nodes after adding start point as obstacle")
        .def("updateEndNeighbor", static_cast<void (VoronoiGraph::*)()>(&VoronoiGraph::getEndNeighbor),
            "Update unique nodes after adding end point as obstacle")
        .def("getStartNeighborNodes", static_cast<const std::vector<VoronoiNode>& (VoronoiGraph::*)() const>(&VoronoiGraph::getStartNeighbor),
            "Get the unique nodes after adding start point as obstacle",
            py::return_value_policy::reference_internal)
        .def("getEndNeighborNodes", static_cast<const std::vector<VoronoiNode>& (VoronoiGraph::*)() const>(&VoronoiGraph::getEndNeighbor),
            "Get the unique nodes after adding end point as obstacle",
            py::return_value_policy::reference_internal);

    py::class_<VoronoiNode>(m, "VoronoiNode")
        .def("getId", &VoronoiNode::getId,
            "Get the node ID")
        .def("getPosition", &VoronoiNode::getPosition,
            "Get the node position")
        .def("getAllAdjacent", &VoronoiNode::getAllAdjacent,
            "Get all adjacent nodes")
        .def("getPathById", &VoronoiNode::getPathById,
            "Get path to a specific node", py::arg("id"));

    py::class_<MapPoint>(m, "MapPoint")
        .def(py::init<int, int>(), py::arg("x"), py::arg("y"))
        .def_readwrite("x", &MapPoint::x)
        .def_readwrite("y", &MapPoint::y);

    py::class_<TrajGenerator, std::shared_ptr<TrajGenerator>>(m, "TrajGenerator")
        .def(py::init())
        .def("initialize", &TrajGenerator::initialize, 
            "Initialize the trajectory generator based on the yaml file", py::arg("map_file"), py::arg("planner_file"), py::arg("path_resolution"), py::arg("time_resolution"))
        .def("sampleTraj", &TrajGenerator::sampleTraj, 
            "Sample a trajectory between two points", py::arg("start"), py::arg("end"))
        .def("sampleDistinctHomotopyTrajs", &TrajGenerator::sampleDistinctHomotopyTrajs, 
            "Sample a trajectory between two points", py::arg("start"), py::arg("end"))
        .def("getInitPlan", &TrajGenerator::getInitPlan,
            "Get the initial plan of the trajectory generator")
        .def("getCircles", &TrajGenerator::getCircles,
            "Get the circles of the trajectory generator") 
        .def("getViaPoints", &TrajGenerator::getViaPoints,
            "Get the via points of the trajectory generator")
        .def("getRawTraj", &TrajGenerator::getRawTraj,
            "Get the trajectory given by teb planner")
        .def("getCostmap", &TrajGenerator::getCostmap,
            "Get the referred static map")
        .def("get_voronoi_graph", &TrajGenerator::getVoronoiGraph,
            "Get the Voronoi graph")
        // .def("get_voronoi_graph_add_start_obstacle", &TrajGenerator::getVoronoiGraphAddStartObstacle,
        //     "Get the Voronoi graph with start point as obstacle")
        // .def("get_voronoi_graph_add_end_obstacle", &TrajGenerator::getVoronoiGraphAddEndObstacle,
        //     "Get the Voronoi graph with end point as obstacle")
        // .def("get_voronoi_graph_add_start_end_obstacle", &TrajGenerator::getVoronoiGraphAddStartEndObstacle,
        //     "Get the Voronoi graph with both start and end points as obstacles")
        .def("generate_modified_voronoi_graph", &TrajGenerator::generateModifiedVoronoiGraph,
            "Generate modified Voronoi graph with start and end points as obstacles", py::arg("start"), py::arg("end"));

    // ============================ only expose when testing teb_local_planner ============================
    // Bind the TebConfig class
    py::class_<TebConfig, std::shared_ptr<TebConfig>>(m, "TebConfig")
        .def(py::init())
        .def("configure", &TebConfig::loadParamsFromYaml, 
            "Configure the teb_local_planner based on the yaml file", py::arg("yaml_filename"));
    
    // Bind the ViaPointContainer class
    py::class_<viaPointContainerClass, std::shared_ptr<viaPointContainerClass>>(m, "ViaPointContainer")
        .def(py::init())
        .def("push_back", &viaPointContainerClass::push_back, 
            "Add a via point to the container", py::arg("x"), py::arg("y"))
        .def("clear", &viaPointContainerClass::clear, 
            "Clear all via points from the container")
        .def("getViaPoints", &viaPointContainerClass::getViaPoints, 
            "Get the via points from the container");

    // Bind the TebOptimalPlanner class
    py::class_<TebOptimalPlanner, std::shared_ptr<TebOptimalPlanner>>(m, "TebOptimalPlanner")
        .def(py::init())
        .def("initialize", 
            [](TebOptimalPlanner& self, const TebConfig& cfg, py::list obstacles, const viaPointContainerClass* via_points = nullptr) {
                // Convert Python list to std::vector<std::shared_ptr<Obstacle>>
                std::shared_ptr<ObstContainer> cxx_obstacles = std::make_shared<ObstContainer>();
                for (auto item : obstacles) {
                    cxx_obstacles->push_back(item.cast<std::shared_ptr<Obstacle>>());
                }
                self.initialize(cfg, cxx_obstacles.get(), via_points ? via_points->getViaPoints() : NULL);
            },
            "Initialize the teb_local_planner",
            py::arg("cfg"), py::arg("obstacles"), py::arg("via_points") = nullptr)
        .def("plan", 
            [](TebOptimalPlanner& self, const PoseSE2& start, const PoseSE2& goal, 
            const std::optional<VelSE2>& start_vel = std::nullopt, bool free_goal_vel = false) {
                return self.plan(start, goal, start_vel ? &(*start_vel) : nullptr, free_goal_vel);
            },
            "Plan a trajectory between a start and goal pose", 
            py::arg("start"), py::arg("goal"), py::arg("start_vel") = py::none(), py::arg("free_goal_vel") = false)
        .def("getFullTrajectory", &TebOptimalPlanner::py_getFullTrajectory, 
            "Get the full trajectory of the teb_local_planner");
    // ==========================================================================================================
}

// Function to path plan
