#include "map_voronoi/voronoinode.h"

void VoronoiNode::addAdjacent(int id, float prob) {
    adjacent.push_back(std::make_pair(id, prob)); // Add an adjacent node with its probability
    activated_adj++; // Increment the count of activated adjacent nodes

    // <可选：对所有邻接点平分概率>
    int adjacent_size = adjacent.size();   
    for (int i = 0; i < adjacent_size; ++i) {
        adjacent[i].second = 1.0f / adjacent_size; // Distribute probability evenly among all adjacent nodes
    }
}

void VoronoiNode::addPath(Path path) {
    pathlist.push_back(path); // Add a path to the node
}

void VoronoiNode::updateProbability(int id) {
    for (auto& adj : adjacent) {
        if (adj.first == id) {
            float prob = adj.second;
            adj.second = 0.0f; // Set the probability of the specified adjacent node to 0
            // int adjacent_size = 1/prob;        // Get the size of the adjacent nodes (When the distribution is not uniform, the size of adjacent nodes is not equal to 1/prob)
            activated_adj--; // Decrement the count of activated adjacent nodes
            for (auto& adj : adjacent) {
                if (adj.second != 0.0f) {
                    adj.second += prob / (activated_adj - 1); // Distribute the probability of the removed node to the remaining nodes
                }
            }
            break;
        }
    }
    LOGGER_WARN("VoronoiNode", "Node %d don't adjacent to %d", node_id, id);
}

void VoronoiNode::resetProbability() {
    int adjacent_size = adjacent.size();
    activated_adj = adjacent_size; // Reset the count of activated adjacent nodes
    for (int i = 0; i < adjacent_size; ++i) {
        adjacent[i].second = 1.0f / adjacent_size; // Distribute probability evenly among all adjacent nodes
    }
}

int VoronoiNode::getAdjacent() {
    // Check if there are adjacent nodes available
    if (adjacent.empty()) {
        std::cerr << "No adjacent nodes available." << std::endl;
        return -1; 
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<float> probabilities;
    for (const auto& pair : adjacent) {
        probabilities.push_back(pair.second);
    }
    std::discrete_distribution<int> dist(probabilities.begin(), probabilities.end());
    int index = dist(gen);

    return adjacent[index].first;
}