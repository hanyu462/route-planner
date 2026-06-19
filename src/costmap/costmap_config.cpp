#include "route_planner/costmap/costmap_config.hpp"

#include <stdexcept>

namespace route_planner::costmap {

void validate_costmap_config(const CostmapConfig& config)
{
    if (config.lethal_radius < 0.0f)
        throw std::invalid_argument("lethal_radius must be >= 0");
    if (config.influence_radius <= config.lethal_radius)
        throw std::invalid_argument("influence_radius must be > lethal_radius");
    if (config.lethal_cost <= config.max_soft_cost)
        throw std::invalid_argument("lethal_cost must be > max_soft_cost");
    if (config.decay_factor <= 0.0f)
        throw std::invalid_argument("decay_factor must be > 0");
}

}  // namespace route_planner::costmap
