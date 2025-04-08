# ifndef _VORONOINODE_H_
# define _VORONOINODE_H_

# include <vector>
# include <utility>
# include <random>
# include <iostream>
# include "voronoi.h"

struct Point{
    int x;
    int y;
};

struct Path{
    int end_node_id; // ID of the end node
    std::vector<Point> path_points; // List of points in the path
};

class VoronoiNode
{
public:
    VoronoiNode(int id, Point position) : node_id(id), node_position(position) {}

    ~VoronoiNode() {}

    Point getPosition() const { return node_position; }
    int getId() const { return node_id; }
    std::vector<std::pair<int, float>> getAllAdjacent() const { return adjacent; }
    // Path getPath() const { return path; }

    void addAdjacent(int id, float prob=1.0f);

    void addPath(Path path);

    // Set the node that has been visited as 0, then update the probability of the left adjacent nodes
    void updateProbability(int id);

    void resetProbability();

    int getAdjacent();  // Get a random adjacent node based on the probability

private:
    int node_id; // ID of the node
    Point node_position; // Position of the node in 2D space
    std::vector<std::pair<int, float>> adjacent; // List of adjacent nodes and their probilities
    std::vector<Path> pathlist; // Path to the node
};


#endif