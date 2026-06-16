#include "route_planner/core/pointcloud_filter.hpp"

namespace route_planner::core {

bool accept_point(
    const PointXYZ& point,
    const PointCloudFilterOptions& options) noexcept
{
    return point.x >= options.min_x && point.x <= options.max_x
        && point.y >= options.min_y && point.y <= options.max_y
        && point.z >= options.min_z && point.z <= options.max_z;
}

}  // namespace route_planner::core
