#pragma once

#include "route_planner/core/pointcloud_transform.hpp"
#include "route_planner/core/pointcloud_filter.hpp"

namespace route_planner::core {

struct PointCloudProcessorConfig {
    PointCloudTransformOptions transform;
    PointCloudFilterOptions    filter;
};

void validate_processor_config(const PointCloudProcessorConfig& config);

}  // namespace route_planner::core
