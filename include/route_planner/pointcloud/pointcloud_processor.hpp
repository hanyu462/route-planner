#pragma once

#include "route_planner/pointcloud/pointcloud_processor_config.hpp"
#include "route_planner/common/pointcloud_xyz_frame.hpp"
#include "route_planner/common/latest_buffer.hpp"

namespace route_planner::pointcloud {

using PointCloudProcessedBuffer = route_planner::common::LatestBuffer<route_planner::common::PointCloudXYZFrame>;

class PointCloudProcessor {
public:
    explicit PointCloudProcessor(PointCloudProcessorConfig config);

    common::PointCloudXYZFrame process(const common::PointCloudXYZFrame& frame) const;

private:
    PointCloudProcessorConfig config_;
};

}  // namespace route_planner::pointcloud
