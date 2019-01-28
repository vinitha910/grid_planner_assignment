#include "graph.h"
#include "dijkstra.h"
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <jsoncpp/json/json.h>

void read_environment(
    const std::string& environment_filename,
    int *width,
    int *height,
    std::vector<int> *occupancy_grid)
{
    std::ifstream file;
    file.open(environment_filename.c_str());
    if (!file.is_open()) return;

    std::string line;
    int num_lines = 0;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string word;
        int num_words = 0;
        while(iss >> word) {
            int occupancy_value = atoi(word.c_str());
            occupancy_grid->push_back(occupancy_value);
            num_words++;
        }
        *width = num_words;
        num_lines++;
    }

    *height = num_lines;
}

void read_solution(
    const std::string& solution_filename,
    std::vector<std::pair<int,int>> *path)
{
    std::ifstream file;
    file.open(solution_filename.c_str());
    if (!file.is_open()) return;

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string word;
        std::vector<int> pair;
        while(iss >> word) {
            int value = atoi(word.c_str());
            pair.push_back(value);
        }
        path->push_back(std::make_pair(pair[0], pair[1]));
    }
}

void print_path(const std::vector<std::pair<int, int>>& path)
{
    for (const auto point : path) {
        std::cout << point.first << " " << point.second << std::endl;
    }
}

void write_solution(
    const std::string& solution_filename,
    const std::vector<std::pair<int, int>>& path)
{
    std::ofstream file;
    file.open(solution_filename.c_str());
    for (const auto point : path) {
        file << point.first << " " << point.second << std::endl;
    }
    file.close();
}

int main() {
    std::ifstream ifs("tests.json");
    Json::Reader reader;
    Json::Value test;
    reader.parse(ifs, test);

    int num_tests_passed = 0;
    for (const auto& test_name : test.getMemberNames()) {
        const std::string environment_filename = test[test_name]["environment"].asString();
        const std::string solution_filename = test[test_name]["solution"].asString();
        const int start_x = test[test_name]["start_x"].asInt();
        const int start_y = test[test_name]["start_y"].asInt();
        const int goal_x = test[test_name]["goal_x"].asInt();
        const int goal_y = test[test_name]["goal_y"].asInt();
        const int true_num_expansions = test[test_name]["num_expansions"].asInt();

        int width, height;
        std::vector<int> occupancy_grid;
        read_environment(environment_filename, &width, &height, &occupancy_grid);
        grid_planner::graphs::Graph graph = \
            grid_planner::graphs::Graph(occupancy_grid, width, height);
        const int start_id = graph.set_start_state(start_x, start_y);
        const int goal_id = graph.set_goal_state(goal_x, goal_y);

        grid_planner::planners::Dijkstras planner =
            grid_planner::planners::Dijkstras(graph);

        std::vector<std::pair<int, int>> path;
        int num_expansions = 0;
        planner.run_planner(start_id, goal_id, &num_expansions, &path);

        std::vector<std::pair<int, int>> true_path;
        read_solution(solution_filename, &true_path);

        if (path == true_path && true_num_expansions == num_expansions) {
            std::cout << test_name << " PASSED!" << std::endl;
        }
        if (path != true_path) {
            std::cout << test_name << " FAILED! -> Incorrect path" << std::endl;
        }
        if (true_num_expansions != num_expansions) {
            std::cout << test_name << " FAILED! -> Incorrect number of expansions" << std::endl;
        }

        write_solution("student_" + solution_filename, path);
    }
    return 1;
}
