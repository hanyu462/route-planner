#pragma once

namespace route_planner::astar {

struct AStarConfig {
    float base_cost;       // base traversal cost per cell (> 0)
    float obs_cost;        // cells with uint8 cost >= this are impassable
    float astar_weight;    // heuristic weight: f = g + w*h  (>= 1.0)
    int   max_iterations;  // iteration cap; returns empty path if exceeded
};

void validate_astar_config(const AStarConfig& config);

}  // namespace route_planner::astar
