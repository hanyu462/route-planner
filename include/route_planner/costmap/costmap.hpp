#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace route_planner::costmap {

struct Costmap {
    int64_t     stamp_ns;
    std::string frame_id;

    float       origin_x;
    float       origin_y;
    float       resolution;
    
    int         width;
    int         height;

    // row-major: cells[row * width + col]
    // 0 = free, 1~253 = soft zone, 254 = lethal (impassable)
    std::vector<uint8_t> cells;
};

}  // namespace route_planner::costmap
