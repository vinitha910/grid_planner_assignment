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
    if (x < 0 || x >= m_width || y < 0 || y >= m_height) {
        return -1;
    }
    m_start_id = get_state_id(x, y);
    if (!is_valid_state(x, y)) {
        return -1;
    }
    return m_start_id;
}

int Graph::set_goal_state(const int& x, const int& y) {
    if (x < 0 || x >= m_width || y < 0 || y >= m_height) {
        return -1;
    }
    m_goal_id = get_state_id(x, y);
    if (!is_valid_state(x, y)) {
        return -1;
    }
    return m_goal_id;
}

void Graph::get_succs(
    const int& source_state_id,
    std::vector<int> *succ_ids,
    std::vector<double> *costs) const {
    assert(source_state_id < m_occupancy_grid.size());
    std::vector<int> succ_temp;
    std::vector<double> costs_temp;
    int x, y;
    Graph::get_coord_from_state_id(source_state_id, &x, &y);
    std::vector<int> x_coords{x - 1, x, x + 1};
    std::vector<int> y_coords{y - 1, y, y + 1};
    for (auto i = x_coords.begin(); i != x_coords.end(); i++) {
        for (auto j = y_coords.begin(); j != y_coords.end(); j++) {
            if (Graph::is_valid_state(*i, *j)) {
                succ_temp.push_back(get_state_id(*i, *j));
                costs_temp.push_back(get_action_cost(x, y, *i, *j));
            }
        }
    }

    *succ_ids = succ_temp;
    *costs = costs_temp;
}

void Graph::get_path_coordinates(
    const std::vector<int>& path_state_ids,
    std::vector<std::pair<int, int> > *path_coordinates) const {
    std::vector<std::pair<int, int>> coordinates;
    for (int i = 0; i < path_state_ids.size(); i++) {
        int x, y;
        bool valid = get_coord_from_state_id(path_state_ids[i], &x, &y);
        assert(valid);
        std::pair<int, int> coord(x, y);
        coordinates.push_back(coord);
    }
    *path_coordinates = coordinates;
}

int Graph::get_state_id(const int& x, const int& y) const {
    assert(x < m_width);
    assert(y < m_height);
    return (y * m_width) + x;
}

bool Graph::get_coord_from_state_id(const int& state_id, int* x, int* y) const {
    assert(state_id < m_occupancy_grid.size());
    *y = state_id / m_width;
    *x = state_id - (*y * m_width);

    return Graph::is_valid_state(*x, *y);
}

bool Graph::is_valid_state(const int& x, const int& y) const {
    if (x < 0 || x >= m_width || y < 0 || y >= m_height) {
        return false;
    }
    int state_id = Graph::get_state_id(x, y);
    return m_occupancy_grid[state_id] == 0;
}

double Graph::get_action_cost(
        const int& source_x,
        const int& source_y,
        const int& succ_x,
        const int& succ_y) const {
    return sqrt(pow(source_x - succ_x, 2) +
            pow(source_y - succ_y, 2));  // euclidean distance
}

}  // namespace graphs
}  // namespace grid_planner

