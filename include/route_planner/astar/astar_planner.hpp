#pragma once

#include <utility>
#include <vector>

#include "route_planner/astar/astar_config.hpp"
#include "route_planner/common/path.hpp"
#include "route_planner/costmap/costmap.hpp"

namespace route_planner::astar {

class AStarPlanner {
public:
    explicit AStarPlanner(AStarConfig config);

    // Plans a path from start_map to goal_map in global map frame.
    // Costmap must already be in global frame (origin_x/y are global coordinates).
    common::Path plan(
        const costmap::Costmap& costmap,
        std::pair<float,float>  start_map,
        std::pair<float,float>  goal_map
    ) const;

private:
    AStarConfig config_;

    std::pair<int,int>     world_to_cell(const costmap::Costmap& m, float x, float y) const;
    std::pair<float,float> cell_to_world(const costmap::Costmap& m, int col, int row) const;
};

}  // namespace route_planner::astar
