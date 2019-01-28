#include "graph.h"
#include <assert.h>
#include <cmath>

namespace grid_planner {
namespace graphs {

int Graph::set_start_state(const int& x, const int& y)
{
    // YOUR CODE HERE
    return -1;
}

int Graph::set_goal_state(const int& x, const int& y)
{
    // YOUR CODE HERE
    return -1;
}

void Graph::get_succs(
    const int& source_state_id,
    std::vector<int> *succ_ids,
    std::vector<double> *costs) const
{
    assert(source_state_id < m_occupancy_grid.size());

    // YOUR CODE HERE
}

void Graph::get_path_coordinates(
    const std::vector<int>& path_state_ids,
    std::vector<std::pair<int, int> > *path_coordinates) const
{
    // YOUR CODE HERE
}

int Graph::get_state_id(const int& x, const int& y) const
{
    assert(x < m_width);
    assert(y < m_height);

    // YOUR CODE HERE
    return 0;
}

bool Graph::get_coord_from_state_id(const int& state_id, int* x, int* y) const
{
    assert(state_id < m_occupancy_grid.size());

    // YOUR CODE HERE
    return true;
}

bool Graph::is_valid_state(const int& x, const int& y) const
{
    // YOUR CODE HERE
    return true;
}

double Graph::get_action_cost(
        const int& source_x,
        const int& source_y,
        const int& succ_x,
        const int& succ_y) const
{
    // YOUR CODE HERE
    return 0;
}

}  // namespace graphs
}  // namespace grid_planner

