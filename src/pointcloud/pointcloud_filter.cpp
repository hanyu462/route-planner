#include "route_planner/pointcloud/pointcloud_filter.hpp"

namespace route_planner::pointcloud {

bool accept_point(
    const common::PointXYZ& point,
    const PointCloudFilterOptions& options) noexcept
{
    const float r2 = options.min_radius * options.min_radius;
    return point.x >= options.min_x && point.x <= options.max_x
        && point.y >= options.min_y && point.y <= options.max_y
        && point.z >= options.min_z && point.z <= options.max_z
        && (point.x * point.x + point.y * point.y) >= r2;
}

}  // namespace route_planner::pointcloud
