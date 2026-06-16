#pragma once

#include "route_planner/core/point_xyz.hpp"

namespace route_planner::core {

struct PointCloudTransformOptions {
    bool enabled;

    float offset_x;
    float offset_y;
    float offset_z;

    bool invert_x;
    bool invert_y;
    bool invert_z;
};

PointXYZ transform_point(
    const PointXYZ& point,
    const PointCloudTransformOptions& options) noexcept;

}  // namespace route_planner::core
