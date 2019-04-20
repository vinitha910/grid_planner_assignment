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

int Graph::set_start_state(const int& x, const int& y) {
    m_start_id = get_state_id(x, y);
    if (is_valid_state(x, y)) {
        return m_start_id;  // valid state
    }

    return -1;  // invalid state
}

int Graph::set_goal_state(const int& x, const int& y) {
    m_goal_id = get_state_id(x, y);
    if (is_valid_state(x, y)) {
        return m_goal_id;  // valid state
    } else {
        return -1;  // invalid state
    }
}

void Graph::get_succs(
    const int& source_state_id,
    std::vector<int> *succ_ids,
    std::vector<double> *costs) const {
    assert(source_state_id < m_occupancy_grid.size());

    int x_source, y_source;
    get_coord_from_state_id(source_state_id, &x_source, &y_source);

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0) {  // current state, not a successor
                continue;
            }

            const int x_succ = x_source + i;
            const int y_succ = y_source + j;

            if (!is_valid_state(x_succ, y_succ)) {
                continue;  // successor is not valid
            }
            const int succ_state_id = get_state_id(x_succ, y_succ);
            (*succ_ids).push_back(succ_state_id);

            // transition cost i.e. cost from parent to successor
            const double succ_cost = get_action_cost(x_source, y_source,
                                                     x_succ, y_succ);
            (*costs).push_back(succ_cost);
        }
    }
}

void Graph::get_path_coordinates(
    const std::vector<int>& path_state_ids,
    std::vector<std::pair<int, int> > *path_coordinates) const {
    for (int index = 0; index < path_state_ids.size(); ++index) {
        const int path_state_id = path_state_ids[index];
        int x, y;
        if (get_coord_from_state_id(path_state_id, &x, &y)) {
            // coordinates are valid
            (*path_coordinates).push_back(std::make_pair(x, y));
        }
    }
}

int Graph::get_state_id(const int& x, const int& y) const {
    assert(x < m_width);
    assert(y < m_height);

    return x + y * m_width;
}

bool Graph::get_coord_from_state_id(const int& state_id, int* x, int* y) const {
    assert(state_id < m_occupancy_grid.size());

    *y = state_id / m_width;
    *x = state_id - *y * m_width;

    return is_valid_state(*x, *y);
}

bool Graph::is_valid_state(const int& x, const int& y) const {
    // check bounds (i.e. check value is valid)
    if (!(x >= 0 && x < m_width && y >= 0 && y < m_height)) {
        return false;
    }
    // check occupancy grid to see if cell is free
    if (m_occupancy_grid[get_state_id(x, y)] == 0) {
        return true;
    }
    return false;
}

double Graph::get_action_cost(
        const int& source_x,
        const int& source_y,
        const int& succ_x,
        const int& succ_y) const {
    // Calculate Euclidean distance between the 2 points
    return sqrt(pow(succ_x - source_x, 2) + pow(succ_y - source_y, 2));
}

}  // namespace graphs
}  // namespace grid_planner

