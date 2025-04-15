# ifndef _VORONOINODE_H_
# define _VORONOINODE_H_

# include <vector>
# include <utility>
# include <random>
# include <iostream>
# include "voronoi.h"
# include "logger.h"

struct MapPoint{
    int x;
    int y;
};

struct Path{
    int end_node_id; // ID of the end node
    std::vector<MapPoint> path_points; // List of points in the path
};

class VoronoiNode
{
public:
    VoronoiNode(int id, MapPoint position) : node_id(id), node_position(position), activated_adj(0)
    {}

    ~VoronoiNode() {}

    MapPoint getPosition() const { return node_position; }
    int getId() const { return node_id; }
    std::vector<std::pair<int, float>> getAllAdjacent() const { return adjacent; }
    const std::vector<MapPoint>& getPathById(int id) const {
        for (const auto& path : pathlist) {
            if (path.end_node_id == id) {
                return path.path_points;
            }
        }
        throw std::out_of_range("Path not found");
    }

    void addAdjacent(int id, float prob=1.0f);

    void addPath(Path path);

    // Set the node that has been visited as 0, then update the probability of the left adjacent nodes
    // return true if node still has adjacent nodes, else return false
    bool deactivate(int id);
    bool activate(int id);
    bool hasAdjacent() const { return activated_adj > 0; } // Check if the node has adjacent nodes

    void resetProbability();

    int getAdjacent();  // Get a random adjacent node based on the probability

    std::vector<int> getNeighbors() const {
        std::vector<int> neighbors;
        for (const auto& adj : adjacent) {
            neighbors.push_back(adj.first);
        }
        return neighbors;
    }

private:
    int node_id; // ID of the node
    MapPoint node_position; // Position of the node in 2D space
    int activated_adj; // Number of activated adjacent nodes
    std::vector<std::pair<int, float>> adjacent; // List of adjacent nodes and their probilities
    std::vector<Path> pathlist; // Path to the node
};


#endif