#ifndef _VORONOIGRAPH_H_
#define _VORONOIGRAPH_H_

#include <vector>
#include <utility>
#include <memory>
#include "voronoi.h"
#include "voronoinode.h"

#include "map_loader/costmap_2d.hpp"
#include "map_loader/cost_values.hpp"
#include "logger.h"

using namespace nav2_costmap_2d;

class VoronoiGraph
{
public:
    VoronoiGraph(std::shared_ptr<Costmap2D> costmap)
    {
        // Initialize the Voronoi object with the costmap
        this->costmap = costmap;
        voronoi_static->initializeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), getBoolMap(costmap));
        voronoi_static->update();
        voronoi_static->updateAlternativePrunedDiagram();  // prune the Voronoi
        // voronoi->mergeVoronoi();

        LOGGER_INFO("VoronoiGraph", "Voronoi graph initialized with size (%d, %d)", costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

        // initialize the static Voronoi graph
        buildGraph();

        LOGGER_INFO("VoronoiGraph", "Voronoi graph built with %zu nodes", voronoi_nodes.size());
    };

    ~VoronoiGraph(){};

    void visualizeVoronoi(const std::string& filename, int type=0); // 0: original, 1: modified

    // get distance to the nearest obstacle (used by the bubble corridor during traj generation)
    float getDistance(int x, int y) { return voronoi_static->getDistance(x, y); }
    // find all available path from start node to end node:
    std::vector<std::vector<int>> findAllPaths(int start_id, int end_id);
    std::vector<int> getPassbyNodes(int start_id, int end_id);

    // @ Old Version
    // get all nodes by pointer (used to find the nearest node to start and end point during traj generation)
    std::vector<VoronoiNode>& getAllNodes() { return voronoi_nodes; }
    const std::vector<VoronoiNode>& getAllNodes() const { return voronoi_nodes; }
    // get node by id
    VoronoiNode& getNodeById(int id) {
        // if (id < 0 || id >= voronoi_nodes.size()) {
        if (id < 0 || id >= voronoi_nodes_modified.size()) {
            throw std::out_of_range("Node ID is out of range");
        }
        // return voronoi_nodes[id];
        return voronoi_nodes_modified[id];
    }
    const VoronoiNode& getNodeById(int id) const {
        if (id < 0 || id >= voronoi_nodes.size()) {
            throw std::out_of_range("Node ID is out of range");
        }
        return voronoi_nodes[id];
    } 

    // @ Bubble technique
    void getVoronoiGraph(unsigned int start_mx, unsigned int start_my, unsigned int end_mx, unsigned int end_my);
    // when using modified voronoi, the start_id and end_id are the id of the start point node and end point node
    int getStartId() const { return start_point_node_id; }
    int getEndId() const { return end_point_node_id; }

private:
    bool** getBoolMap(std::shared_ptr<Costmap2D> costmap);
    void buildGraph();
    void resetAllProbabilities();

    std::shared_ptr<Costmap2D> costmap;
    std::shared_ptr<Voronoi> voronoi_static = std::make_shared<Voronoi>();
    std::vector<VoronoiNode> voronoi_nodes; // List of Voronoi nodes

    // @ Bubble technique
    std::shared_ptr<Voronoi> voronoi_modified = std::make_shared<Voronoi>();  // Bubble technique (see the start and end as obstacles)
    int start_point_node_id, end_point_node_id; // start and end node id
    std::vector<VoronoiNode> voronoi_nodes_modified; // List of Voronoi nodes
    std::vector<VoronoiNode> voronoi_nodes_startNeighbor;
    std::vector<VoronoiNode> voronoi_nodes_endNeighbor;

    void getStartNeighbor();
    void getEndNeighbor();
    const std::vector<VoronoiNode>& getStartNeighbor() const { return voronoi_nodes_startNeighbor; }
    const std::vector<VoronoiNode>& getEndNeighbor() const { return voronoi_nodes_endNeighbor; }

};

#endif
