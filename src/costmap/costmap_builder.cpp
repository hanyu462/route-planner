#include "route_planner/costmap/costmap_builder.hpp"

#include <algorithm>
#include <cmath>

#include "route_planner/costmap/euclidean_distance_transform.hpp"

namespace route_planner::costmap {

// ── Cost LUT ──────────────────────────────────────────────────────────────────

void CostmapBuilder::build_lut(float resolution) const
{
    lut_resolution_ = resolution;

    // Maximum cell distance that can carry any cost
    const int max_cells =
        static_cast<int>(std::ceil(config_.influence_radius / resolution));
    const int max_sq = (max_cells + 1) * (max_cells + 1);

    cost_lut_.resize(static_cast<std::size_t>(max_sq + 1));

    for (int d2 = 0; d2 <= max_sq; ++d2) {
        const float dist_m = std::sqrt(static_cast<float>(d2)) * resolution;

        uint8_t cost;
        if (dist_m <= config_.lethal_radius) {
            cost = config_.lethal_cost;
        } else if (dist_m >= config_.influence_radius) {
            cost = uint8_t{0};
        } else {
            const float raw =
                static_cast<float>(config_.max_soft_cost) *
                std::exp(-config_.decay_factor * (dist_m - config_.lethal_radius));
            cost = static_cast<uint8_t>(
                std::clamp(static_cast<int>(raw), 0,
                           static_cast<int>(config_.max_soft_cost)));
        }

        cost_lut_[static_cast<std::size_t>(d2)] = cost;
    }
}

// ── CostmapBuilder ────────────────────────────────────────────────────────────

CostmapBuilder::CostmapBuilder(CostmapConfig config)
    : config_(config)
{}

Costmap CostmapBuilder::build(const occupancy_grid::OccupancyGrid& grid) const
{
    if (cost_lut_.empty() || lut_resolution_ != grid.resolution) {
        build_lut(grid.resolution);
    }

    const std::vector<float> dist_sq =
        compute_edt_sq(grid.cells, grid.width, grid.height);

    Costmap costmap;
    costmap.stamp_ns   = grid.stamp_ns;
    costmap.frame_id   = grid.frame_id;
    costmap.origin_x   = grid.origin_x;
    costmap.origin_y   = grid.origin_y;
    costmap.resolution = grid.resolution;
    costmap.width      = grid.width;
    costmap.height     = grid.height;

    const int lut_max = static_cast<int>(cost_lut_.size()) - 1;
    const int N       = grid.width * grid.height;
    costmap.cells.resize(static_cast<std::size_t>(N));

    for (int i = 0; i < N; ++i) {
        const int d2 = static_cast<int>(
            std::min(dist_sq[static_cast<std::size_t>(i)],
                     static_cast<float>(lut_max)));
        costmap.cells[static_cast<std::size_t>(i)] =
            cost_lut_[static_cast<std::size_t>(d2)];
    }

    return costmap;
}

}  // namespace route_planner::costmap
