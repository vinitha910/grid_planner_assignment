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

namespace grid_planner {
namespace graphs {

int Graph::set_start_state(const int& x, const int& y)
{
    // YOUR CODE HERE
/*
    m_start_id = get_state_id(x, y);
    
    if (is_valid_state(m_start_id)) {
      return m_start_id;  // valid state
    } else {
      return -1;  // invalid state
    }
*/

    return -1;
}

int Graph::set_goal_state(const int& x, const int& y)
{
    // YOUR CODE HERE
/*
    m_goal_state = get_state_id(x, y);

    if (is_valid_state(m_goal_state)) {
      return m_goal_state;  // valid state
    } else {
      return -1;  // invalid state
    }
*/
    return -1;
}

void Graph::get_succs(
    const int& source_state_id,
    std::vector<int> *succ_ids,
    std::vector<double> *costs) const
{
    assert(source_state_id < m_occupancy_grid.size());

    // YOUR CODE HERE
/*  // HOW DO I GET THE SUCCESSORS?
    std::vector<int>::iterator iter;
    for (iter = (*succ_ids).begin(); iter != (*succ_ids).end(); iter++) {
      if (is_valid_state(*iter)) {  // successor is valid
        
      }
    }
*/
}

void Graph::get_path_coordinates(
    const std::vector<int>& path_state_ids,
    std::vector<std::pair<int, int> > *path_coordinates) const
{
    // YOUR CODE HERE
/*
    std::vector<int>::iterator iter;
    for (iter = path_state_ids.begin(); iter != path_state_ids.end(); iter++) {
      int x, y;
      if (get_coord_from_state_id(*iter, &x, &y) {  //IS *iter CORRECT HERE?
        (*path_coordinates).push_back(x, y);  //  coordinates are valid
      }
    }
*/
}

int Graph::get_state_id(const int& x, const int& y) const
{
    assert(x < m_width);
    assert(y < m_height);

    // YOUR CODE HERE
/*
    
*/
    return 0;
}

bool Graph::get_coord_from_state_id(const int& state_id, int* x, int* y) const
{
    assert(state_id < m_occupancy_grid.size());

    // YOUR CODE HERE
/*

*/
    return true;
}

bool Graph::is_valid_state(const int& x, const int& y) const
{
    // YOUR CODE HERE
/*

*/
    return true;
}

double Graph::get_action_cost(
        const int& source_x,
        const int& source_y,
        const int& succ_x,
        const int& succ_y) const
{
    // YOUR CODE HERE
/*
    // Are we calculating distance b/w the 2 points?
    return sqrt(pow(succ_x - source_x, 2) + pow(succ_y - source_y, 2));
*/
    return 0;
}

}  // namespace graphs
}  // namespace grid_planner

