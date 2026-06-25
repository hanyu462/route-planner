#include "route_planner/astar/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace route_planner::astar {

AStarPlanner::AStarPlanner(AStarConfig config)
    : config_(config)
{}

std::pair<int,int> AStarPlanner::world_to_cell(
    const costmap::Costmap& m, float x, float y) const
{
    return {
        static_cast<int>((x - m.origin_x) / m.resolution),
        static_cast<int>((y - m.origin_y) / m.resolution)
    };
}

std::pair<float,float> AStarPlanner::cell_to_world(
    const costmap::Costmap& m, int col, int row) const
{
    return {
        m.origin_x + (col + 0.5f) * m.resolution,
        m.origin_y + (row + 0.5f) * m.resolution
    };
}

common::Path AStarPlanner::plan(
    const costmap::Costmap& costmap,
    std::pair<float,float>  start_map,
    std::pair<float,float>  goal_map
) const
{
    const int W = costmap.width;
    const int H = costmap.height;
    const int N = W * H;

    const auto [sc, sr] = world_to_cell(costmap, start_map.first, start_map.second);
    const auto [gc, gr] = world_to_cell(costmap, goal_map.first,  goal_map.second);

    auto in_bounds = [&](int c, int r) {
        return c >= 0 && c < W && r >= 0 && r < H;
    };

    common::Path path;
    path.stamp_ns = costmap.stamp_ns;
    path.frame_id = costmap.frame_id;

    if (!in_bounds(sc, sr) || !in_bounds(gc, gr)) return path;

    const int   start_idx = sr * W + sc;
    const int   goal_idx  = gr * W + gc;
    const auto  obs       = static_cast<uint8_t>(config_.obs_cost);

    if (start_idx == goal_idx) return path;
    if (costmap.cells[start_idx] >= obs || costmap.cells[goal_idx] >= obs) return path;

    constexpr float kInf = std::numeric_limits<float>::infinity();
    std::vector<float> g(N, kInf);
    std::vector<int>   parent(N, -1);
    std::vector<bool>  closed(N, false);

    using Entry = std::pair<float, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    g[start_idx] = 0.0f;
    const float h0 = config_.astar_weight *
        static_cast<float>(std::max(std::abs(gc - sc), std::abs(gr - sr))) *
        config_.base_cost;
    open.push({h0, start_idx});

    // 8-connected neighbours: delta_col, delta_row, move_distance_multiplier
    static constexpr int   dc[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static constexpr int   dr[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    static constexpr float dm[] = {
        1.41421356f, 1.0f, 1.41421356f,
        1.0f,              1.0f,
        1.41421356f, 1.0f, 1.41421356f
    };

    int  iterations = 0;
    bool found      = false;

    while (!open.empty() && iterations < config_.max_iterations) {
        const auto [f_curr, idx] = open.top();
        open.pop();
        (void)f_curr;

        if (closed[idx]) continue;
        closed[idx] = true;
        ++iterations;

        if (idx == goal_idx) { found = true; break; }

        const int r = idx / W;
        const int c = idx % W;

        for (int d = 0; d < 8; ++d) {
            const int nc = c + dc[d];
            const int nr = r + dr[d];
            if (!in_bounds(nc, nr)) continue;

            const int nidx = nr * W + nc;
            if (closed[nidx]) continue;

            const uint8_t cell_cost = costmap.cells[nidx];
            if (cell_cost >= obs) continue;

            const float step = (config_.base_cost + static_cast<float>(cell_cost)) * dm[d];
            const float ng   = g[idx] + step;

            if (ng < g[nidx]) {
                g[nidx]      = ng;
                parent[nidx] = idx;
                const float h = config_.astar_weight *
                    static_cast<float>(std::max(std::abs(gc - nc), std::abs(gr - nr))) *
                    config_.base_cost;
                open.push({ng + h, nidx});
            }
        }
    }

    if (!found) return path;

    for (int idx = goal_idx; idx != -1; idx = parent[idx]) {
        path.waypoints.push_back(cell_to_world(costmap, idx % W, idx / W));
    }
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    return path;
}

}  // namespace route_planner::astar
