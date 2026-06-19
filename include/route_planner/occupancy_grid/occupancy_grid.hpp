#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace route_planner::occupancy_grid {

struct OccupancyGrid {
    int64_t     stamp_ns;
    std::string frame_id;

    float origin_x;    // world X coordinate of the grid's bottom-left corner (m)
    float origin_y;    // world Y coordinate of the grid's bottom-left corner (m)
    float resolution;  // meters per cell

    int width;   // number of cells along X (columns)
    int height;  // number of cells along Y (rows)

    // row-major: cells[row * width + col], 0=free 100=occupied
    std::vector<int8_t> cells;
};

}  // namespace route_planner::occupancy_grid
