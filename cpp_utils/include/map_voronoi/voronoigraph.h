#ifndef _VORONOIGRAPH_H_
#define _VORONOIGRAPH_H_

#include <vector>
#include <utility>
#include <memory>
#include "voronoi.h"
#include "voronoinode.h"
#include "map_loader/costmap_2d.hpp"

using namespace nav2_costmap_2d;

class VoronoiGraph
{
public:
    VoronoiGraph(){

    }
    ~VoronoiGraph(){

    }

    void visualizeVoronoi(const std::string& filename);
    void getVoronoiGraph(int sizeX, int sizeY, bool** map);

private:
    Costmap2D* costmap;  // Pointer to the costmap object
    std::shared_ptr<Voronoi> voronoi = std::make_shared<Voronoi>();
    std::vector<VoronoiNode> voronoi_nodes; // List of Voronoi nodes

};

#endif
