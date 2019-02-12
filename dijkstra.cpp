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
#include <iostream>
namespace grid_planner {
namespace planners {

void Dijkstras::run_planner(
    const int& start_id,
    const int& goal_id,
    int* num_expansions,
    std::vector<std::pair<int, int>> *path)
{
    // Create priority queue; I suggest using a set with the custom
    // comparator defined in dijkstra.h as your priority queue
   
    std::set<int> Q; // You will need to change this line

/*
    std::set<int, CostMapComparator> Q;
*/

    // While the queue is not empty
    while (!Q.empty()) {
        // Pop and expand the next node in the priority queue
        (*num_expansions)++;

        // YOUR CODE HERE
/*
        
*/
    }
}

void Dijkstras::extract_path(
    const ChildToParentMap& child_to_parent_map,
    const int& start_id,
    const int& goal_id,
    std::vector<int> *path_state_ids)
{
    // YOUR CODE HERE
/*
    int x, y;

//I think I have to get the path backwards (i.e. from goal to start)

// Can end if we have expanded the goal or we have nothing in our priority list.
/
    if (goal_id == start_id) {
      return;
    }

    if (get coord_from_state_id(goal_id, &x, &y) {
      // push goal_id
      //check if x and why represents valid state?
      (*path_state_ids).push_back(goal_id);
    }

    //ChildToParentMap is typedef for std::unordered_map<int, int>!
    //maps child state id to parent state id
    // what is the child_to parent_map doing exactly?
    ChildToParentMap::iterator iter = child_to_parent_map.find(goal_id);

    while (1) {  // we can put end condition here!!!!!
      if (iter != child_to_parent_map.end()) {
        // Should I remove the mapping?
        std::string value = iter->second;

        // do I need to check if the state id is valid?
        (*path_state_ids).push_back(value);
        child_to_parent_map.erase(iter);  // removes mapping from map if key is found

        // will this work for the case when the start and goal are the same?

        if (value == start_id) {
          break;
        }
      } else {
        break;
      }

      iter = child_to_parent_map.find(value);
    }
*/
}

}
}
