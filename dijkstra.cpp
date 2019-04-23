/////////////////////////////////////////////////////////////////////////////
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

namespace grid_planner {
namespace planners {

void Dijkstras::run_planner(
    const int& start_id,
    const int& goal_id,
    int* num_expansions,
    std::vector<std::pair<int, int>> *path) {

    CostMap cost_map;
    CostMapComparator cost_map_comparator(cost_map);
    std::set<int, CostMapComparator> Q(cost_map_comparator);

    cost_map[start_id] = 0;
    Q.insert(start_id);

    ChildToParentMap child_to_parent_map;

    std::vector<int> path_state_ids;

    // While the queue is not empty
    while (!Q.empty()) {
        (*num_expansions)++;

        const int parent_id = *(Q.begin());
        Q.erase(Q.begin());

        if (parent_id == goal_id) {
            extract_path(child_to_parent_map, start_id, goal_id,
                         &path_state_ids);
            m_graph.get_path_coordinates(path_state_ids, path);
            return;
        }

        std::vector<int> succ_ids;
        std::vector<double> costs;
        m_graph.get_succs(parent_id, &succ_ids, &costs);

        assert(succ_ids.size() == costs.size());

        for (int index = 0; index < succ_ids.size(); ++index) {
            const int succ_state_id = succ_ids[index];
            const double transition_cost = costs[index];
            const double g_value = cost_map[parent_id] + transition_cost;
            if (cost_map.find(succ_state_id) == cost_map.end() ||
                g_value < cost_map[succ_state_id]) {
                cost_map[succ_state_id] = g_value;
                Q.erase(succ_state_id);
                Q.insert(succ_state_id);
                child_to_parent_map[succ_state_id] = parent_id;
            }
        }
    }

    // if Q is empty, we need to call extract_path
    extract_path(child_to_parent_map, start_id, goal_id, &path_state_ids);

    // set path parameter
    m_graph.get_path_coordinates(path_state_ids, path);
}

void Dijkstras::extract_path(
    const ChildToParentMap& child_to_parent_map,
    const int& start_id,
    const int& goal_id,
    std::vector<int> *path_state_ids) {

    if (goal_id == start_id) {
        return;
    }

    (*path_state_ids).push_back(goal_id);

    auto path_iter = child_to_parent_map.find(goal_id);
    // loop till we find start or we reach end of map
    while (path_iter != child_to_parent_map.end()) {
        (*path_state_ids).push_back(path_iter->second);
        path_iter = child_to_parent_map.find(path_iter->second);
    }

    std::reverse(path_state_ids->begin(), path_state_ids->end());
}
}  // namespace planners

}  // namespace grid_planner
