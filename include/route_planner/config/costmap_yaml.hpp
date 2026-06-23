#pragma once

#include <string>

#include "route_planner/costmap/costmap_config.hpp"

namespace route_planner::config {

costmap::CostmapConfig load_costmap_config(const std::string& yaml_path);

}  // namespace route_planner::config
