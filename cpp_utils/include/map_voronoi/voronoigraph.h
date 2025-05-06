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
        voronoi->initializeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), getBoolMap(costmap));
        voronoi->update();
        voronoi->updateAlternativePrunedDiagram();  // prune the Voronoi
        voronoi->mergeVoronoi();
    };

    ~VoronoiGraph(){};

    void visualizeVoronoi(const std::string& filename);
    void getVoronoiGraph();

    // TODO: get bool map from costmap according to the cost values
    bool** getBoolMap(std::shared_ptr<Costmap2D> costmap);

    float getDistance(int x, int y) { return voronoi->getDistance(x, y); }

    // get all nodes by pointer
    const std::vector<VoronoiNode>& getAllNodes() const {return voronoi_nodes;}

    // get node by id
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

private:
    std::shared_ptr<Voronoi> voronoi = std::make_shared<Voronoi>();
    std::vector<VoronoiNode> voronoi_nodes; // List of Voronoi nodes

};

#endif
