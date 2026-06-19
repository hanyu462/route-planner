#include "route_planner/occupancy_grid/occupancy_grid_builder.hpp"

#include <cmath>

namespace route_planner::occupancy_grid {

OccupancyGridBuilder::OccupancyGridBuilder(OccupancyGridConfig config)
    : config_(config)
{
    origin_x_ = config_.min_x;
    origin_y_ = config_.min_y;
    width_  = static_cast<int>(std::floor((config_.max_x - config_.min_x) / config_.resolution));
    height_ = static_cast<int>(std::floor((config_.max_y - config_.min_y) / config_.resolution));
}

OccupancyGrid OccupancyGridBuilder::build(
    const common::PointCloudXYZFrame& frame) const
{
    OccupancyGrid grid;
    grid.stamp_ns   = frame.stamp_ns;
    grid.frame_id   = frame.frame_id;
    grid.origin_x   = origin_x_;
    grid.origin_y   = origin_y_;
    grid.resolution = config_.resolution;
    grid.width      = width_;
    grid.height     = height_;
    grid.cells.assign(static_cast<std::size_t>(width_ * height_), config_.cell_values.free);

    const std::size_t n = frame.point_count();
    for (std::size_t i = 0; i < n; ++i) {
        const float x = frame.xyz[i * 3 + 0];
        const float y = frame.xyz[i * 3 + 1];

        const int col = static_cast<int>(std::floor((x - origin_x_) / config_.resolution));
        const int row = static_cast<int>(std::floor((y - origin_y_) / config_.resolution));

        if (col >= 0 && col < width_ && row >= 0 && row < height_) {
            grid.cells[static_cast<std::size_t>(row * width_ + col)] = config_.cell_values.occupied;
        }
    }

    return grid;
}

}  // namespace route_planner::occupancy_grid
