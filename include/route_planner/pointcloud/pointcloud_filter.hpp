#pragma once

#include "route_planner/common/point_xyz.hpp"

namespace route_planner::pointcloud {

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
    const common::PointXYZ& point,
    const PointCloudFilterOptions& options) noexcept;

}  // namespace route_planner::pointcloud
