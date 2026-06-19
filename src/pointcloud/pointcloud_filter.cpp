#include "route_planner/pointcloud/pointcloud_filter.hpp"

namespace route_planner::pointcloud {

bool accept_point(
    const common::PointXYZ& point,
    const PointCloudFilterOptions& options) noexcept
{
    if (!options.enabled) return true;

    if (options.enable_x && (point.x < options.min_x || point.x > options.max_x)) return false;
    if (options.enable_y && (point.y < options.min_y || point.y > options.max_y)) return false;
    if (options.enable_z && (point.z < options.min_z || point.z > options.max_z)) return false;

    if (options.enable_radius) {
        const float r2 = options.min_radius * options.min_radius;
        if ((point.x * point.x + point.y * point.y) < r2) return false;
    }

    return true;
}

}  // namespace route_planner::pointcloud
