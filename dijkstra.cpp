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

#include "dijkstra.h"
#include <set>
#include <cassert>
#include <algorithm>
#include <float.h>
#include <iostream>
namespace grid_planner {
namespace planners {

void Dijkstras::run_planner(
    const int& start_id,
    const int& goal_id,
    int* num_expansions,
    std::vector<std::pair<int, int>> *path) {
    CostMap cost_map;
    cost_map[start_id] = 0;
    ChildToParentMap child_to_parent_map;
    CostMapComparator cost_map_comparator(cost_map);

    // Create priority queue; I suggest using a set with with the custom
    // comparator defined in dijkstra.h as your priority queue
    std::set<int, CostMapComparator> Q(cost_map_comparator);
    std::set<int> visited;
    Q.insert(start_id);

    // While the queue is not empty
    while (!Q.empty()) {
        // Pop and expand the next node in the priority queue
        (*num_expansions)++;
        std::set<int>::iterator it = Q.begin();
        Q.erase(it);
        int node = *it;
        visited.insert(node);
        if (node == goal_id) {
            break;
        }

        std::vector<int> succ_ids;
        std::vector<double> costs;
        m_graph.get_succs(node, &succ_ids, &costs);
        for (int i = 0; i != succ_ids.size(); i++) {
            double updatedDistance = cost_map[node] + costs[i];
            bool seenNode = visited.find(succ_ids[i]) != visited.end();
            if (!seenNode || (updatedDistance < cost_map[succ_ids[i]])) {
                cost_map[succ_ids[i]] = updatedDistance;
                child_to_parent_map[succ_ids[i]] = node;
                Q.insert(succ_ids[i]);
                visited.insert(succ_ids[i]);
            }
        }
    }

    std::vector<int> path_state_ids;
    extract_path(child_to_parent_map, start_id, goal_id, &path_state_ids);
    m_graph.get_path_coordinates(path_state_ids, path);
}

void Dijkstras::extract_path(
    const ChildToParentMap& child_to_parent_map,
    const int& start_id,
    const int& goal_id,
    std::vector<int> *path_state_ids) {
    std::vector<int> path;
    int parent = goal_id;
    while (parent != start_id) {
        parent = child_to_parent_map.at(parent);
        path.push_back(parent);
    }
    std::reverse(path.begin(), path.end());
    path.push_back(goal_id);
    *path_state_ids = path;
}

}
}
