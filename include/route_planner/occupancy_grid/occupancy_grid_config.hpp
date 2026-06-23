#pragma once

#include <cstdint>

namespace route_planner::occupancy_grid {

struct CellValues {
    int8_t unknown;   // -1 by convention
    int8_t free;      // 0~100
    int8_t occupied;  // 0~100, must differ from free
};

struct OccupancyGridConfig {
    float      resolution;       // meters per cell, > 0
    CellValues cell_values;

    float min_x;
    float max_x;
    float min_y;
    float max_y;
};

void validate_occupancy_grid_config(const OccupancyGridConfig& config);

}  // namespace route_planner::occupancy_grid
