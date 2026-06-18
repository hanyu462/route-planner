#pragma once

#include "route_planner/pointcloud/pointcloud_transform.hpp"
#include "route_planner/pointcloud/pointcloud_filter.hpp"

namespace route_planner::pointcloud {

struct PointCloudProcessorConfig {
    PointCloudTransformOptions transform;
    PointCloudFilterOptions    filter;
};

void validate_processor_config(const PointCloudProcessorConfig& config);

}  // namespace route_planner::pointcloud
