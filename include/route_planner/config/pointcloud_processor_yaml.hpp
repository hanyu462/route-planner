#pragma once

#include <string>

#include "route_planner/pointcloud/pointcloud_processor_config.hpp"

namespace route_planner::config {

pointcloud::PointCloudProcessorConfig load_processor_config(const std::string& yaml_path);

}  // namespace route_planner::config
