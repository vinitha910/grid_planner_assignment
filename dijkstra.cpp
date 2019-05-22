////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that g following conditions are met:
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

#include "./dijkstra.h"
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
    std::vector<std::pair<int, int>> *path) {

    CostMap map;
    ChildToParentMap child_to_parent_map;
    CostMapComparator comparator(map);
    std::set<int, CostMapComparator> Q(comparator);
    std::vector<int> path_ids;
    Q.insert(start_id);
    map[start_id] = 0;
    while (!Q.empty()) {
        (*num_expansions)++;
        int curr_state = *(Q.begin());
        Q.erase(Q.begin());
        Q.erase(curr_state);
        if (curr_state  == goal_id) {
            extract_path(child_to_parent_map, start_id, goal_id, &path_ids);
            m_graph.get_path_coordinates(path_ids, path);
            return;
        }
        std::vector<int> succesor_ids;
        std::vector<double> costs;
        m_graph.get_succs(curr_state, &succesor_ids, &costs);
        for (int idx = 0; idx < succesor_ids.size(); ++idx) {
            int succ_id = succesor_ids[idx];
            double new_cost = map[curr_state] + costs[idx];
            if (map.find(succ_id) == map.end() || map[succ_id] > new_cost) {
                child_to_parent_map[succ_id] = curr_state;
                map[succ_id] = new_cost;
                assert(Q.find(curr_state) == Q.end());
                Q.erase(succ_id);
                Q.insert(succ_id);
            }
        }
        assert(Q.find(curr_state) == Q.end());
    }
    extract_path(child_to_parent_map, start_id, goal_id, &path_ids);
    m_graph.get_path_coordinates(path_ids, path);
}

void Dijkstras::extract_path(
    const ChildToParentMap& child_to_parent_map,
    const int& start_id,
    const int& goal_id,
    std::vector<int> *path_state_ids) {
    if (goal_id == start_id) {
        return;
    }

    assert(child_to_parent_map.find(goal_id) != child_to_parent_map.end());
    auto parent = child_to_parent_map.find(goal_id);
    path_state_ids->push_back(goal_id);
    while (parent != child_to_parent_map.end()) {
        path_state_ids->push_back(parent->second);
        parent = child_to_parent_map.find(parent->second);
    }
    std::reverse(path_state_ids->begin(), path_state_ids->end());
}
}  // namespace planners
}  // namespace grid_planner
