#pragma once

#include <cstdint>
#include <string>

namespace route_planner::common {

struct PoseXY {
    int64_t stamp_ns;
    std::string frame_id;
    float x;
    float y;
    float qx;
    float qy;
    float qz;
    float qw;
};

}  // namespace route_planner::common
