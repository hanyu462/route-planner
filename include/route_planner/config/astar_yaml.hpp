#pragma once

#include <string>

#include "route_planner/astar/astar_config.hpp"

namespace route_planner::config {

astar::AStarConfig load_astar_config(const std::string& yaml_path);

}  // namespace route_planner::config
