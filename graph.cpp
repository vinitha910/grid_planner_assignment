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

#include "graph.h"
#include <assert.h>
#include <cmath>
#include <iostream>

namespace grid_planner {
namespace graphs {

int Graph::set_start_state(const int& x, const int& y) {
    // YOUR CODE HERE

    m_start_id = get_state_id(x, y);

    if (is_valid_state(x, y)) {
      return m_start_id;  // valid state
    } else {
      return -1;  // invalid state
    }

    // END OF MY CODE

//    return -1;
}

int Graph::set_goal_state(const int& x, const int& y) {
    // YOUR CODE HERE

    m_goal_id = get_state_id(x, y);

    if (is_valid_state(x, y)) {
      return m_goal_id;  // valid state
    } else {
      return -1;  // invalid state
    }

    // END OF MY CODE

//    return -1;
}

void Graph::get_succs(
    const int& source_state_id,
    std::vector<int> *succ_ids,
    std::vector<double> *costs) const {

    assert(source_state_id < m_occupancy_grid.size());

    // YOUR CODE HERE
/*
    std::vector<int>::iterator iter;
    for (iter = (*succ_ids).begin(); iter != (*succ_ids).end(); iter++) {
      if (is_valid_state(*iter)) {  // successor is valid
        
      }
    }
*/
    int x_source, y_source;
    get_coord_from_state_id(source_state_id, &x_source, &y_source);
    int x_succ, y_succ;
    int succ_state_id;
    double succ_cost;

    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        if (i == 0 && j == 0) {  // current state, not a successor
          continue;
    }

    x_succ = x_source + i;
    y_succ = y_source + j;
    // succ_state_id = get_state_id(x_succ, y_succ);

    if (!is_valid_state(x_succ, y_succ)) {
          continue;  // successor is not valid
    } else {
        succ_state_id = get_state_id(x_succ, y_succ);
        (*succ_ids).push_back(succ_state_id);
        // transition cost i.e. cost from parent to successor
        succ_cost = get_action_cost(x_source, y_source, x_succ, y_succ);
        (*costs).push_back(succ_cost);

       // std::cout << succ_state_id << " = " << succ_cost << std::endl;
    }
      }
    }

    // END OF MY CODE
}

void Graph::get_path_coordinates(
    const std::vector<int>& path_state_ids,
    std::vector<std::pair<int, int> > *path_coordinates) const {
    // YOUR CODE HERE

    for (auto iter = path_state_ids.begin(); iter != path_state_ids.end(); iter++) {
      int x, y;
      if (get_coord_from_state_id(*iter, &x, &y)) {
        (*path_coordinates).push_back(std::make_pair(x, y));  //  coordinates are valid
      }
    }

    // END OF MY CODE
}

int Graph::get_state_id(const int& x, const int& y) const {
    assert(x < m_width);
    assert(y < m_height);

    // YOUR CODE HERE

    return x + y * m_width;

    // END OF MY CODE

    // return 0;
}

bool Graph::get_coord_from_state_id(const int& state_id, int* x, int* y) const {
    assert(state_id < m_occupancy_grid.size());

    // YOUR CODE HERE

    *y = state_id / m_width;
    *x = state_id - *y * m_width;

    return is_valid_state(*x, *y);

    // END OF MY CODE

    return true;
}

bool Graph::is_valid_state(const int& x, const int& y) const {
    // YOUR CODE HERE

    // check bounds (i.e. value is valid)
    if (!(x >= 0 && x < m_width && y >= 0 && y < m_height)) {
      return false;
    } else {  // check occupancy grid to see if cell is free
       if (m_occupancy_grid[get_state_id(x, y)] == 0) {
         return true;
       } else {
         return false;
       }
    }

    // END OF MY CODE

    return true;
}

double Graph::get_action_cost(
        const int& source_x,
        const int& source_y,
        const int& succ_x,
        const int& succ_y) const {
    // YOUR CODE HERE

    // Calculate Euclidean distance between the 2 points
    return sqrt(pow(succ_x - source_x, 2) + pow(succ_y - source_y, 2));

    // END OF MY CODE
    //   return 0;
}

}  // namespace graphs
}  // namespace grid_planner

