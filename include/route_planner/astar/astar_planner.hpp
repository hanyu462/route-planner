#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "route_planner/astar/astar_config.hpp"
#include "route_planner/common/path.hpp"
#include "route_planner/common/pose_xy.hpp"
#include "route_planner/costmap/costmap.hpp"

namespace route_planner::astar {

class AStarPlanner {
public:
    explicit AStarPlanner(AStarConfig config);

    // Plans a path from robot origin to goal_map in map frame.
    // If pose is provided, converts between map frame and costmap-local frame.
    common::Path plan(
        const costmap::Costmap& costmap,
        std::pair<float,float>  goal_map,
        const std::optional<common::PoseXY>& pose
    ) const;

private:
    AStarConfig config_;

    // Core A* in costmap-local frame. Returns local-frame waypoints.
    std::vector<std::pair<float,float>> plan_local(
        const costmap::Costmap& costmap,
        float start_x, float start_y,
        float goal_x,  float goal_y
    ) const;

    std::pair<int,int>     world_to_cell(const costmap::Costmap& m, float x, float y) const;
    std::pair<float,float> cell_to_world(const costmap::Costmap& m, int col, int row) const;

    static float extract_yaw(const common::PoseXY& pose);
};

}  // namespace route_planner::astar
