#include "route_planner/occupancy_grid/occupancy_grid_config.hpp"

#include <stdexcept>

namespace route_planner::occupancy_grid {

void validate_occupancy_grid_config(const OccupancyGridConfig& config)
{
    if (config.resolution <= 0.0f)
        throw std::invalid_argument("occupancy_grid.resolution must be > 0");

    const auto& cv = config.cell_values;
    if (cv.free < 0 || cv.free > 100)
        throw std::invalid_argument("cell_values.free must be between 0 and 100");
    if (cv.occupied < 0 || cv.occupied > 100)
        throw std::invalid_argument("cell_values.occupied must be between 0 and 100");
    if (cv.free == cv.occupied)
        throw std::invalid_argument("cell_values.free and cell_values.occupied must differ");

    if (config.min_x >= config.max_x)
        throw std::invalid_argument("occupancy_grid.min_x must be < max_x");

    if (config.min_y >= config.max_y)
        throw std::invalid_argument("occupancy_grid.min_y must be < max_y");
}

}  // namespace route_planner::occupancy_grid
