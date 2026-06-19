#pragma once

#include <string>

#include "route_planner/occupancy_grid/occupancy_grid_config.hpp"

namespace route_planner::config {

occupancy_grid::OccupancyGridConfig load_occupancy_grid_config(const std::string& yaml_path);

}  // namespace route_planner::config
