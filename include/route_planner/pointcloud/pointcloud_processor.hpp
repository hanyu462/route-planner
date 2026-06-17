#pragma once

#include "route_planner/pointcloud/pointcloud_processor_config.hpp"
#include "route_planner/common/pointcloud_xyz_frame.hpp"

namespace route_planner::pointcloud {

class PointCloudProcessor {
public:
    explicit PointCloudProcessor(PointCloudProcessorConfig config);

    common::PointCloudXYZFrame process(const common::PointCloudXYZFrame& frame) const;

private:
    PointCloudProcessorConfig config_;
};

}  // namespace route_planner::pointcloud
