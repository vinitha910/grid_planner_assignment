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
   
//    std::set<int> Q; // You will need to change this line
/*
// MY CODE BEGINS
    // set of state IDs ordered by cost (rather then sorted in order of integers)
    std::set<int, CostMapComparator> Q;

    std::vector<int> path_state_ids; //close list?
// MY CODE ENDS
*/
    // While the queue is not empty
    while (!Q.empty()) {
        // Pop and expand the next node in the priority queue
        (*num_expansions)++;
/*
        // YOUR CODE HERE
	const auto iterator node = Q.begin();

	if (*node == goal_id) {
          extract_path(ChildToParentMap, start_id, goal_id, &path_state_ids);
	  // set path parameter?
	  break;
	}

        std::vector<int> succ_ids;
        std::vector<double> costs;
        get_succ(start_id, &succ_ids, &costs);
        auto iterStateID = succ_ids.begin();
	auto iterCosts = costs.begin();
	while (iterStateID != succ_ids.end() && iterCosts != succ_ids.end()) {
          // find cost
	  int x_parent, y_parent, x_succ, y_succ;
	  get_coord_from_state_id(*node, &x_parent, &y_parent); //gets parents coordinates
          get_coord_from_state_id(*iterStateID, &x_succ, &y_succ); //gets succ coordinates
          double gValue = get_action_cost(x_parent, y_parent, x_succ, y_succ); //finds cost from parent to successor
          gValue += cost_map_[*node]; // adds above to parent's cost (i.e. from parent to start_id) to get g value

          // if node is not in the priority queue, we need to add it!
	  if (Q.find(node) == Q.end()) {
            Q.insert(node);
	  } else {
            // node is in priority queue, but update it to have optimal cost
	    // For Dikstras, optimal cost is the least cost
	    if (gValue < cost_map_[node]) { //if (CostMapComparator(*iterStateID, *node)) {
              // update cost map?
	      cost_map_[node] = gValue;
	    }
	  }
	}
      }

      // if Q is empty, we need to call extract_path too
      if (Q.empty()) {
        extract_path(ChildToParentMap, start_id, goal_id, &path_state_ids);
      }

      // set path parameter
      for (auto iter = path_state_ids.begin(); iter != path_state_ids.end(); ++iter) {
        int x, y;
	get_coord_from_state_id(*iter, &x, &y);
        path.pushback(std::make_pair(x, y));
      }
*/
/*
        check if node popped is goal
	 call extract_path
	 break

        //popped node has optimal cost

        call get successors function

	//close list has nodes with optimal cost
	// open list has everything that in in priority queue, and nodes that haven't been seen yet
	
	// check if sucessor node is in the priority queue
	// if its not in the priority queue, we need to add it
	//     find cost from parent to successor by calling get_action _cost
	//     and add this to parent's cost to get g value
	//
	// else
	//    find cost here too
	//    
	//    update cost to optimal cost in priority queue
	//    
	//
	//at end of while loop, call extract path
	
// we can stop if we have expanded the goal or we have nothing in our priority queue
  */      

        // MY CODE ENDS

}

void Dijkstras::extract_path(
    const ChildToParentMap& child_to_parent_map,
    const int& start_id,
    const int& goal_id,
    std::vector<int> *path_state_ids)
{
  /*  // YOUR CODE HERE

    int x, y;

//I think I have to get the path backwards (i.e. from goal to start)

// Can stop if we have expanded the goal or we have nothing in our priority list.

//do find and check if value is end before loop!

    if (goal_id == start_id) {
      return;
    }

    if (get coord_from_state_id(goal_id, &x, &y)) {
      // check if x and y represent valid state
      // push goal_id
      (*path_state_ids).push_back(goal_id);
    }

    //ChildToParentMap is typedef for std::unordered_map<int, int>!
    //maps child state id to parent state id
    // what is the child_to parent_map doing exactly?
    ChildToParentMap::iterator iter = child_to_parent_map.find(goal_id);
    (*path_state_ids).push_back(goal_id);

    // loop till we find start or we reach end of map
    while (iter->second != start_id && iter != child_to_parent_map.end()) {
      (*path_state_ids).push_back(iter->second);
      iter = child_to_parent_map.find(iter->second);
    }
*/
/* OLD CODE
    while (1) {  // we can put end condition here!!!!!
      if (iter != child_to_parent_map.end()) {
        // Should I remove the mapping?
        std::string value = iter->second;

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
    // END OF MY CODE
}

}
}
