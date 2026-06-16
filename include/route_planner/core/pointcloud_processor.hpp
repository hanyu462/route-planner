#pragma once

#include "route_planner/core/pointcloud_processor_config.hpp"
#include "route_planner/core/pointcloud_xyz_frame.hpp"

namespace route_planner::core {

class PointCloudProcessor {
public:
    explicit PointCloudProcessor(PointCloudProcessorConfig config);

    PointCloudXYZFrame process(const PointCloudXYZFrame& frame) const;

private:
    PointCloudProcessorConfig config_;
};

}  // namespace route_planner::core
