#include "route_planner/occupancy_grid/occupancy_grid_builder.hpp"

#include <cmath>

namespace route_planner::occupancy_grid {

OccupancyGridBuilder::OccupancyGridBuilder(OccupancyGridConfig config)
    : config_(config)
{
    width_  = static_cast<int>(std::floor((config_.max_x - config_.min_x) / config_.resolution));
    height_ = static_cast<int>(std::floor((config_.max_y - config_.min_y) / config_.resolution));
}

OccupancyGrid OccupancyGridBuilder::build(
    const common::PointCloudXYZFrame& frame,
    const common::PoseXY& pose) const
{
    OccupancyGrid grid;
    grid.stamp_ns   = frame.stamp_ns;
    grid.frame_id   = pose.frame_id;
    grid.origin_x   = pose.x + config_.min_x;
    grid.origin_y   = pose.y + config_.min_y;
    grid.resolution = config_.resolution;
    grid.width      = width_;
    grid.height     = height_;
    grid.cells.assign(static_cast<std::size_t>(width_ * height_), config_.cell_values.free);

    const float yaw     = std::atan2(
        2.0f * (pose.qw * pose.qz + pose.qx * pose.qy),
        1.0f - 2.0f * (pose.qy * pose.qy + pose.qz * pose.qz));
    const float cos_yaw = std::cos(yaw);
    const float sin_yaw = std::sin(yaw);

    const std::size_t n = frame.point_count();
    for (std::size_t i = 0; i < n; ++i) {
        const float lx = frame.xyz[i * 3 + 0];
        const float ly = frame.xyz[i * 3 + 1];

        const float gx = pose.x + cos_yaw * lx - sin_yaw * ly;
        const float gy = pose.y + sin_yaw * lx + cos_yaw * ly;

        const int col = static_cast<int>(std::floor((gx - grid.origin_x) / config_.resolution));
        const int row = static_cast<int>(std::floor((gy - grid.origin_y) / config_.resolution));

        if (col >= 0 && col < width_ && row >= 0 && row < height_) {
            grid.cells[static_cast<std::size_t>(row * width_ + col)] = config_.cell_values.occupied;
        }
    }

    return grid;
}

}  // namespace route_planner::occupancy_grid
