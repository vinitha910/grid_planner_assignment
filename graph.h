#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <utility>

namespace grid_planner {
namespace graphs {

// This class implements a 2D grid-based graph representation
class Graph {
 public:
    // occupancy_grid A 1D vector containing either 1 for an occupied/obstacle
    // cell or 0 for a free cell. The index is a 1D representation for an (x, y)
    // width The width of the occupancy grid
    // height The height of the occupancy grid
    Graph(const std::vector<int>& occupancy_grid,
          const int& width,
          const int& height) : \
          m_occupancy_grid(occupancy_grid),
          m_width(width),
          m_height(height) {}

    ~Graph() {};

    // Sets and returns the start state ID (m_start_id)
    // Returns -1 if the state is not valid
    int set_start_state(const int& x, const int& y);

    // Sets and returns the goal state ID (m_goal_id)
    // Returns -1 if the state is not valid
    int set_goal_state(const int& x, const int& y);

    // Finds the valid successors and transition costs for the source state.
    // The vaild successor IDs and costs are filled into succ_ids and costs
    // respectively
    void get_succs(
        const int& source_state_id,
        std::vector<int> *succ_ids,
        std::vector<double> *costs) const;

    // Find the coordinates for the path given a vector containing all the
    // state IDs
    void get_path_coordinates(
        const std::vector<int>& path_state_ids,
        std::vector<std::pair<int, int> > *path_coordinates) const;

 private:
    // Returns the state ID (1D representation) for the given (x, y) cell
    int get_state_id(const int& x, const int& y) const;

    // Gets the coordinates for the given state ID and stores then in x and y
    // Return true if coordinates are valid and false otherwise
    bool get_coord_from_state_id(const int& state_id, int* x, int* y) const;

    // Return true if the state is valid and false otherwise
    bool is_valid_state(const int& x, const int& y) const;

    // Returns the cost of transitioning from (source_x, source_y) to
    // (succ_x, succ_y)
    double get_action_cost(
        const int& source_x,
        const int& source_y,
        const int& succ_x,
        const int& succ_y) const;

    const std::vector<int> m_occupancy_grid;
    const int m_width;
    const int m_height;

    int m_start_id;
    int m_goal_id;
};

}
}

#endif
