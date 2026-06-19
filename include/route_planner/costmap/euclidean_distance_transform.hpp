#pragma once

#include <cstdint>
#include <vector>

namespace route_planner::costmap {

// Felzenszwalb & Huttenlocher (2012) exact 2-D Euclidean Distance Transform.
// Returns squared distance in cell units for every grid cell.
// cells: row-major flat array, occupied_value marks obstacle cells.
std::vector<float> compute_edt_sq(
    const std::vector<int8_t>& cells,
    int width,
    int height,
    int8_t occupied_value = int8_t{100});

}  // namespace route_planner::costmap
