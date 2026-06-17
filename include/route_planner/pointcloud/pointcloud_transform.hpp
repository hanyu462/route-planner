#pragma once

#include "route_planner/common/point_xyz.hpp"

namespace route_planner::pointcloud {

struct PointCloudTransformOptions {
    bool enabled;

    float offset_x;
    float offset_y;
    float offset_z;

    bool invert_x;
    bool invert_y;
    bool invert_z;
};

common::PointXYZ transform_point(
    const common::PointXYZ& point,
    const PointCloudTransformOptions& options) noexcept;

}  // namespace route_planner::pointcloud
