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
    //pair(x,y)
    std::vector<std::pair<int, int>> *path)
{
    
    //define costmap
    CostMap map;
    ChildToParentMap child_to_parent_map;
    // Create priority queue; I suggest using a set with with the custom
    // comparator defined in dijkstra.h as your priority queue
    CostMapComparator comparator(map);
    std::set<int, CostMapComparator> Q(comparator); // You will need to change this line
    
    // While the queue is not empty
    
    while (!Q.empty()) {
        // Pop and expand the next node in the priority queue
        (*num_expansions)++;

        std::set<int, CostMapComparator>::iterator curr_state = Q.begin();
        std::vector<int> *succesor_ids;
        std::vector<double> *costs;
        

        if (*curr_state == start_id) 
        {
            map.insert(std::pair<int, double> (start_id, 0));
        }
        if (*curr_state  == goal_id) {

            std::vector<int> *path_ids; 
            extract_path(child_to_parent_map, start_id, goal_id, path_ids);
            for (int i = 0; i < path_ids->size(); i++) {
                //get_coordinate from state_id at each iteration
                //convert coordinate into std::pair
                //insert pair into *path
                m_graph.get_path_coordinates(*path_ids,path);

            }
            break;
        }
        m_graph.get_succs(*curr_state, succesor_ids, costs);
        //for each succesor:
        //    check if in costmap: yes -> compare, no -> add to map with cost
        for (int i = 0; i < succesor_ids->size(); i++)
        {
            int curr_id = succesor_ids->at(i);
            //add to map (succesor, currrent state)
            //child: succesor, parent: currr
            //hence child to parent map
            child_to_parent_map.insert(std::make_pair(curr_id, *curr_state));

            CostMap::iterator map_cost = map.find(curr_id);
            double succ_cost = costs->at(i);
            double tmp_cost = succ_cost = map_cost->second + succ_cost;
            if (map.find(curr_id) != map.end()) {
                
                if (succ_cost < map_cost->second) {
                    map.insert(std::pair<int, double>(curr_id,succ_cost));
                }
            } else {
                map.insert(std::pair<int, double>(curr_id, succ_cost + map_cost->second));
            }
        }
        ++curr_state;

    }
}

void Dijkstras::extract_path(
    const ChildToParentMap& child_to_parent_map,
    const int& start_id,
    const int& goal_id,
    std::vector<int> *path_state_ids)
{
    //will return path from start  to goal
    int start = goal_id;
    path_state_ids->push_back(start);
    while(start != start_id){
        int child = child_to_parent_map.at(start);
        path_state_ids->push_back(child);
        start =child;
    }
}

}
}
