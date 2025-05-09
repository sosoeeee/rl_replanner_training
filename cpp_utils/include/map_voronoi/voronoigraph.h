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
        voronoi->initializeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), getBoolMap(costmap));
        voronoi->update();
        voronoi->updateAlternativePrunedDiagram();  // prune the Voronoi
        // voronoi->mergeVoronoi();
    };

    void updateVoronoiGraph(std::shared_ptr<Costmap2D> costmap_modified, const MapPoint& point) {
        // add point as the obstacle:
        voronoi_modified->initializeMap(costmap_modified->getSizeInCellsX(), costmap_modified->getSizeInCellsY(), getBoolMap(costmap_modified));
        voronoi_modified->update();
        voronoi_modified->updateAlternativePrunedDiagram();  // prune the Voronoi
    }

    ~VoronoiGraph(){};

    void visualizeVoronoi(const std::string& filename);
    void visualizeVoronoiModified(const std::string& filename);
    void getVoronoiGraph();
    void getModifiedVoronoiGraph();

    // TODO: get bool map from costmap according to the cost values
    bool** getBoolMap(std::shared_ptr<Costmap2D> costmap);

    // bool** getModifiedBoolMap(std::shared_ptr<Costmap2D> costmap, const MapPoint& point);

    float getDistance(int x, int y) { return voronoi->getDistance(x, y); }

    // get all nodes by pointer
    std::vector<VoronoiNode>& getAllNodes() { return voronoi_nodes; }
    std::vector<VoronoiNode>& getAllNodesModified() { return voronoi_nodes_modified; }
    const std::vector<VoronoiNode>& getAllNodes() const { return voronoi_nodes; }

    // get node by id
    VoronoiNode& getNodeById(int id) {
        if (id < 0 || id >= voronoi_nodes.size()) {
            throw std::out_of_range("Node ID is out of range");
        }
        return voronoi_nodes[id];
    }
    const VoronoiNode& getNodeById(int id) const {
        if (id < 0 || id >= voronoi_nodes.size()) {
            throw std::out_of_range("Node ID is out of range");
        }
        return voronoi_nodes[id];
    } 

    // find all available path from start node to end node:
    std::vector<std::vector<int>> findAllPaths(int start_id, int end_id);

    void resetAllProbabilities();
    std::vector<int> getPassbyNodes(int start_id, int end_id);

    void getStartNeighbor();
    void getEndNeighbor();

    // 获取起点相邻的节点
    const std::vector<VoronoiNode>& getStartNeighbor() const { return voronoi_nodes_startNeighbor; }
    
    // 获取终点相邻的节点
    const std::vector<VoronoiNode>& getEndNeighbor() const { return voronoi_nodes_endNeighbor; }

private:
    std::shared_ptr<Costmap2D> costmap;
    std::shared_ptr<Voronoi> voronoi = std::make_shared<Voronoi>();
    std::shared_ptr<Voronoi> voronoi_modified = std::make_shared<Voronoi>();
    std::vector<VoronoiNode> voronoi_nodes; // List of Voronoi nodes
    std::vector<VoronoiNode> voronoi_nodes_modified;
    std::vector<VoronoiNode> voronoi_nodes_startNeighbor;
    std::vector<VoronoiNode> voronoi_nodes_endNeighbor;

};

#endif
