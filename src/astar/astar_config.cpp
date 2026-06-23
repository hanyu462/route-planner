#include "route_planner/astar/astar_config.hpp"

#include <stdexcept>

namespace route_planner::astar {

void validate_astar_config(const AStarConfig& config)
{
    if (config.base_cost <= 0.0f)
        throw std::invalid_argument("astar.base_cost must be > 0");
    if (config.obs_cost <= 0.0f)
        throw std::invalid_argument("astar.obs_cost must be > 0");
    if (config.astar_weight < 1.0f)
        throw std::invalid_argument("astar.astar_weight must be >= 1.0");
    if (config.max_iterations <= 0)
        throw std::invalid_argument("astar.max_iterations must be > 0");
}

}  // namespace route_planner::astar
