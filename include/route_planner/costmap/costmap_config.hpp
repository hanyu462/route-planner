#pragma once

#include <cstdint>

namespace route_planner::costmap {

struct CostmapConfig {
    float   lethal_radius;    // absolute forbidden zone in metres (robot footprint + safety)
    float   influence_radius; // soft-zone extent in metres, must be > lethal_radius
    uint8_t lethal_cost;      // cost value for lethal zone (impassable)
    uint8_t max_soft_cost;    // cost value at lethal boundary, decays toward 0
    float   decay_factor;     // exponential decay rate: cost = max_soft_cost * exp(-k*(d-lethal_r))
};

void validate_costmap_config(const CostmapConfig& config);

}  // namespace route_planner::costmap
