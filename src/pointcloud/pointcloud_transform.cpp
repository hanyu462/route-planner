#include "route_planner/pointcloud/pointcloud_transform.hpp"

namespace route_planner::pointcloud {

common::PointXYZ transform_point(
    const common::PointXYZ& point,
    const PointCloudTransformOptions& options) noexcept
{
    const float x = options.invert_x ? -point.x : point.x;
    const float y = options.invert_y ? -point.y : point.y;
    const float z = options.invert_z ? -point.z : point.z;

    return common::PointXYZ{
        options.offset_x + x,
        options.offset_y + y,
        options.offset_z + z
    };
}

}  // namespace route_planner::pointcloud
