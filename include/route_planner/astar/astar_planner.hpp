#pragma once

#include <utility>
#include <vector>

#include "route_planner/astar/astar_config.hpp"
#include "route_planner/costmap/costmap.hpp"

namespace route_planner::astar {

class AStarPlanner {
public:
    explicit AStarPlanner(AStarConfig config);

    // Plans a path from (start_x, start_y) to (goal_x, goal_y) in costmap frame.
    // Returns a vector of (x, y) world-coordinate waypoints, or empty if no path found.
    std::vector<std::pair<float,float>> plan(
        const costmap::Costmap& costmap,
        float start_x, float start_y,
        float goal_x,  float goal_y
    ) const;

private:
    AStarConfig config_;

    std::pair<int,int>     world_to_cell(const costmap::Costmap& m, float x, float y) const;
    std::pair<float,float> cell_to_world(const costmap::Costmap& m, int col, int row) const;
};

}  // namespace route_planner::astar
