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
#include <iostream>
namespace grid_planner {
namespace planners {

void Dijkstras::run_planner(
    const int& start_id,
    const int& goal_id,
    int* num_expansions,
    std::vector<std::pair<int, int>> *path)
{
    //std::cout << "in run planner" << std::endl;
    // Create priority queue; I suggest using a set with the custom
    // comparator defined in dijkstra.h as your priority queue
   
//    std::set<int> Q; // You will need to change this line

// MY CODE BEGINS
    // set of state IDs ordered by cost (rather then sorted in order of integers)
    CostMap cost_map;
    CostMapComparator cost_map_comparator(cost_map);
    std::set<int, CostMapComparator> Q(cost_map_comparator);
    
    cost_map[start_id] = 0;
    //std::cout << start_id << " = " << cost_map[start_id] << std::endl;
    Q.insert(start_id);
    // cost_map[start_id] = 0;

    std::vector<int> path_state_ids; //close list?

    ChildToParentMap child_to_parent_map; // empty map
// MY CODE ENDS

    // While the queue is not empty
    while (!Q.empty()) {
        //std::cout << "getchar" << std::endl;
        //getchar();  // debugging technique that works on Vinitha's laptop!
        // Pop and expand the next node in the priority queue
        (*num_expansions)++;

        // YOUR CODE HERE
	    const int parent = *(Q.begin()); 
        Q.erase(Q.begin());

	    if (parent == goal_id) {
            extract_path(child_to_parent_map, start_id, goal_id, &path_state_ids);
	        // set path parameter?
            m_graph.get_path_coordinates(path_state_ids, path);
            return;
	    }

        std::vector<int> succ_ids;
        std::vector<double> costs;
        m_graph.get_succs(parent, &succ_ids, &costs);
        auto iterStateID = succ_ids.begin();
	    auto iterCosts = costs.begin();
	    
        while (iterStateID != succ_ids.end() && iterCosts != costs.end()) {
            //std::cout << "Succs in Planner function" << std::endl;
            //std::cout << *iterStateID << " = " << *iterCosts << std::endl;
            // child_to_parent_map[*iterStateID] = parent;

            double gValue = cost_map[parent] + *iterCosts; //finds cost from parent to successor
            // gValue += cost_map[*node]; //cost_map_[*node]; // adds above to parent's cost (i.e. from parent to start_id) to get g value
            
            // if node is not in the priority queue, we need to add it
            // check cost map instead of doing Q.find (which does binary search on the set so it uses the comparator, which leads to undefined behavior when the cost hasn't been defined for a stateID
            if (cost_map.find(*iterStateID) == cost_map.end()) {
                cost_map[*iterStateID] = gValue;
                Q.insert(*iterStateID);
                child_to_parent_map[*iterStateID] = parent;
            } else if (gValue < cost_map[*iterStateID]) {
                cost_map[*iterStateID] = gValue;
                //std::cout << *iterStateID << " = " << cost_map[*iterStateID] << std::endl;
                Q.erase(*iterStateID); // do this because we can't insert a duplicate value! we need to erase the stateID and then insert it again with the new cost; the comparator will only be called when we do find() or insert()
                Q.insert(*iterStateID);
                child_to_parent_map[*iterStateID] = parent;
                //std::cout << "==========================" << std::endl;
                // cost_map[*iterStateID] = gValue; 
            }
            
            /*
            if (Q.find(*iterStateID) == Q.end()) {
                std::cout << "node not in priority queue" << std::endl;
                //std::cout << *iterStateID << " = " << cost_map[*iterStateID] << std::endl;
                cost_map[*iterStateID] = gValue;
                std::cout << *iterStateID << " = " << cost_map[*iterStateID] << std::endl;
                Q.insert(*iterStateID);
                std::cout << "==========================" << std::endl;
                // cost_map[*iterStateID] = gValue;
	        } else {
                std::cout << "node is in priority queue" << std::endl;
                // node is in priority queue, but update it to have optimal cost
	            // For Dikstras, optimal cost is the least cost
	            if (gValue < cost_map[*iterStateID]) { //cost_map_[node]) { //if (CostMapComparator(*iterStateID, *node)) {
                    // update cost map?
	                cost_map[*iterStateID] = gValue; // cost_map_[node] = gValue;
	            }
	        }
            */
            ++iterStateID;
            ++iterCosts;
	    }
    }

      // if Q is empty, we need to call extract_path too
      if (Q.empty()) {
        extract_path(child_to_parent_map, start_id, goal_id, &path_state_ids);
      }

      // set path parameter
      m_graph.get_path_coordinates(path_state_ids, path);

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
// YOUR CODE HERE

    int x, y;

//I think I have to get the path backwards (i.e. from goal to start)
// Can stop if we have expanded the goal or we have nothing in our priority list.

//do find and check if value is end before loop!

    if (goal_id == start_id) {
      return;
    }

    std::cout << "start_id = " << start_id << std::endl;
    std::cout << "goal_id = " << goal_id << std::endl;

    (*path_state_ids).push_back(goal_id);

    //ChildToParentMap is typedef for std::unordered_map<int, int>!
    //maps child state id to parent state id
    // what is the child_to parent_map doing exactly?
    auto iter = child_to_parent_map.find(goal_id);

    // loop till we find start or we reach end of map
    while (iter != child_to_parent_map.end()) {

      //getchar();
      //std::cout << iter->second << std::endl;
      (*path_state_ids).push_back(iter->second);
      iter = child_to_parent_map.find(iter->second);
    }

    //(*path_state_ids).push_back(start_id);

    
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
    std::reverse(path_state_ids->begin(), path_state_ids->end());

    // END OF MY CODE
  }
}

}

