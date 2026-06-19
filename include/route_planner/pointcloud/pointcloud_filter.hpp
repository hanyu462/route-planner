#pragma once

#include "route_planner/common/point_xyz.hpp"

namespace route_planner::pointcloud {

struct PointCloudFilterOptions {
    bool enabled;

    bool  enable_x;
    float min_x;
    float max_x;

    bool  enable_y;
    float min_y;
    float max_y;

    bool  enable_z;
    float min_z;
    float max_z;

    bool  enable_radius;
    float min_radius;  // XY 평면 기준 원점 최소 거리 (이 반경 내 포인트 제거)
};

bool accept_point(
    const common::PointXYZ& point,
    const PointCloudFilterOptions& options) noexcept;

}  // namespace route_planner::pointcloud
