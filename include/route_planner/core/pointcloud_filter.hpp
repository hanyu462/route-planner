#pragma once

#include "route_planner/core/point_xyz.hpp"

namespace route_planner::core {

struct PointCloudFilterOptions {
    bool enabled;

    float min_x;
    float max_x;

    float min_y;
    float max_y;

    float min_z;
    float max_z;
};

bool accept_point(
    const PointXYZ& point,
    const PointCloudFilterOptions& options) noexcept;

}  // namespace route_planner::core
