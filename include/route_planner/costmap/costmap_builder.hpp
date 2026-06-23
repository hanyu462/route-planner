#pragma once

#include <cstdint>
#include <vector>

#include "route_planner/costmap/costmap.hpp"
#include "route_planner/costmap/costmap_config.hpp"
#include "route_planner/occupancy_grid/occupancy_grid.hpp"

namespace route_planner::costmap {

class CostmapBuilder {
public:
    explicit CostmapBuilder(CostmapConfig config);

    Costmap build(const occupancy_grid::OccupancyGrid& grid) const;

private:
    CostmapConfig config_;

    // LUT: index = squared cell distance → uint8 cost.
    // Built on first build() call; rebuilt if grid resolution changes.
    mutable std::vector<uint8_t> cost_lut_;
    mutable float                lut_resolution_ = 0.0f;

    void build_lut(float resolution) const;
};

}  // namespace route_planner::costmap
