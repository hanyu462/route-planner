#include "route_planner/pointcloud/pointcloud_filter.hpp"

namespace route_planner::pointcloud {

bool accept_point(
    const common::PointXYZ& point,
    const PointCloudFilterOptions& options) noexcept
{
    return point.x >= options.min_x && point.x <= options.max_x
        && point.y >= options.min_y && point.y <= options.max_y
        && point.z >= options.min_z && point.z <= options.max_z;
}

}  // namespace route_planner::pointcloud
