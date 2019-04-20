////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#ifndef DIJKSTRA_H_
#define DIJKSTRA_H_

#include <vector>
#include <utility>
#include <unordered_map>
#include "graph.h"

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
        return cost_map_.find(state_1)->second <= cost_map_.find(state_2)->second;
    }

 private:
    const CostMap& cost_map_;
};

// This class implements dijkstra's algorithm
class Dijkstras {
 public:
    Dijkstras(
        const graphs::Graph& graph) : m_graph(graph) {}

    ~Dijkstras() {}

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

}  // namespace planners
}  // namespace grid_planner

#endif  // DIJKSTRA_H_
