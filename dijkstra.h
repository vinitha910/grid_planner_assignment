#ifndef DIJKSTRAS_H_
#define DIJKSTRAS_H_

#include "graph.h"
#include <vector>
#include <utility>
#include <unordered_map>

namespace grid_planner {
namespace planners {

// Maps state ID to it's appropiate cost
typedef std::unordered_map<int, double> CostMap;

// Maps the child/successor state ID to the parent ID
typedef std::unordered_map<int, int> ChildToParentMap;

// Comparator used to order the states based on their costs
class CostMapComparator {
 public:
    explicit CostMapComparator(const CostMap& cost_map): cost_map_(cost_map) {}

    bool operator()(const int& state_1,
                    const int& state_2) const {
        // Given two states you need to write a comparator that determines
        // how to order them
        // YOUR CODE HERE (replace line below)
        return true;
    }

 private:
    const CostMap& cost_map_;
};

// This class implements dijkstra's algorithm
class Dijkstras {
 public:

    Dijkstras(
        const graphs::Graph& graph) : m_graph(graph) {}

    ~Dijkstras() {};

    // Runs the planner from the start ID to the goal ID and fills in the
    // final path states into the path vector
    void run_planner(
        const int& start_id,
        const int& goal_id,
        int* num_expansions,
        std::vector<std::pair<int, int>> *path);

 private:
    // Extracts the path of stateIDs from the start to the goal ID
    void extract_path(
        const ChildToParentMap& child_to_parent_map,
        const int& start_id,
        const int& goal_id,
        std::vector<int> *path_state_ids);

    const graphs::Graph m_graph;

};

}
}

#endif
