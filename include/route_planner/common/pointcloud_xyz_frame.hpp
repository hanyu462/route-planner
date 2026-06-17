#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace route_planner::common {

struct PointCloudXYZFrame {
    int64_t stamp_ns = 0;
    std::string frame_id;

    // float32[N, 3] — memory layout: x0 y0 z0 x1 y1 z1 ...
    std::vector<float> xyz;

    std::size_t point_count() const noexcept {
        return xyz.size() / 3;
    }

    bool empty() const noexcept {
        return xyz.empty();
    }
};

}  // namespace route_planner::common
