#include "dijkstra.h"
#include <set>
#include <cassert>
#include <algorithm>
#include <iostream>
namespace grid_planner {
namespace planners {

void Dijkstras::run_planner(
    const int& start_id,
    const int& goal_id,
    int* num_expansions,
    std::vector<std::pair<int, int>> *path)
{
    // Create priority queue; I suggest using a set with a std::set with the
    // custom comparator defined in dijkstra.h
    std::set<int> Q; // You will need to change this line

    // While the queue is not empty
    while (!Q.empty()) {
        // Pop and expand the next node in the queue
        (*num_expansions)++;

        // YOUR CODE HERE
    }
}

void Dijkstras::extract_path(
    const ChildToParentMap& child_to_parent_map,
    const int& start_id,
    const int& goal_id,
    std::vector<int> *path_state_ids)
{
    // YOUR CODE HERE
}

}
}
