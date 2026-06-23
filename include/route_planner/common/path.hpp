#pragma once

#include <string>
#include <utility>
#include <vector>

#include "route_planner/common/latest_buffer.hpp"

namespace route_planner::common {

struct Path {
    int64_t     stamp_ns;
    std::string frame_id;
    std::vector<std::pair<float,float>> waypoints;  // (x, y) in frame_id
};

using PathBuffer = LatestBuffer<Path>;

}  // namespace route_planner::common
