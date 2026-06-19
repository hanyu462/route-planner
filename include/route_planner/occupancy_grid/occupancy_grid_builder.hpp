#pragma once

#include "route_planner/common/latest_buffer.hpp"
#include "route_planner/common/pointcloud_xyz_frame.hpp"
#include "route_planner/occupancy_grid/occupancy_grid.hpp"
#include "route_planner/occupancy_grid/occupancy_grid_config.hpp"

namespace route_planner::occupancy_grid {

using OccupancyGridBuffer = common::LatestBuffer<OccupancyGrid>;

class OccupancyGridBuilder {
public:
    explicit OccupancyGridBuilder(OccupancyGridConfig config);

    OccupancyGrid build(const common::PointCloudXYZFrame& frame,
                        float robot_x = 0.0f, float robot_y = 0.0f) const;

private:
    OccupancyGridConfig config_;
    float origin_x_;
    float origin_y_;
    int   width_;
    int   height_;
};

}  // namespace route_planner::occupancy_grid
