#include "route_planner/astar/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace route_planner::astar {

AStarPlanner::AStarPlanner(AStarConfig config)
    : config_(config)
{}

float AStarPlanner::extract_yaw(const common::PoseXY& p)
{
    return std::atan2(
        2.0f * (p.qw * p.qz + p.qx * p.qy),
        1.0f - 2.0f * (p.qy * p.qy + p.qz * p.qz));
}

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
    const costmap::Costmap&              costmap,
    std::pair<float,float>               goal_map,
    const std::optional<common::PoseXY>& pose
) const
{
    common::Path path;
    path.stamp_ns = costmap.stamp_ns;
    path.frame_id = pose ? pose->frame_id : costmap.frame_id;

    float goal_local_x = goal_map.first;
    float goal_local_y = goal_map.second;

    if (pose) {
        const float yaw = extract_yaw(*pose);
        const float dx  = goal_map.first  - pose->x;
        const float dy  = goal_map.second - pose->y;
        goal_local_x    =  std::cos(yaw) * dx + std::sin(yaw) * dy;
        goal_local_y    = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    }

    auto local_waypoints = plan_local(costmap, 0.0f, 0.0f, goal_local_x, goal_local_y);

    if (pose && !local_waypoints.empty()) {
        const float yaw = extract_yaw(*pose);
        path.waypoints.reserve(local_waypoints.size());
        for (const auto& [lx, ly] : local_waypoints) {
            path.waypoints.emplace_back(
                pose->x + std::cos(yaw) * lx - std::sin(yaw) * ly,
                pose->y + std::sin(yaw) * lx + std::cos(yaw) * ly);
        }
    } else {
        path.waypoints = std::move(local_waypoints);
    }

    if (!path.waypoints.empty()) {
        const auto& first = path.waypoints.front();
        const auto& last  = path.waypoints.back();
        std::printf("[T6] path: %zu waypoints  start=(%.2f, %.2f)  goal=(%.2f, %.2f)\n",
                    path.waypoints.size(),
                    first.first, first.second,
                    last.first,  last.second);
    } else {
        std::printf("[T6] No path found (goal_local: %.2f, %.2f)\n",
                    goal_local_x, goal_local_y);
    }

    return path;
}

std::vector<std::pair<float,float>> AStarPlanner::plan_local(
    const costmap::Costmap& costmap,
    float start_x, float start_y,
    float goal_x,  float goal_y
) const
{
    const int W = costmap.width;
    const int H = costmap.height;
    const int N = W * H;

    const auto [sc, sr] = world_to_cell(costmap, start_x, start_y);
    const auto [gc, gr] = world_to_cell(costmap, goal_x,  goal_y);

    auto in_bounds = [&](int c, int r) {
        return c >= 0 && c < W && r >= 0 && r < H;
    };

    if (!in_bounds(sc, sr) || !in_bounds(gc, gr)) return {};

    const int   start_idx = sr * W + sc;
    const int   goal_idx  = gr * W + gc;
    const auto  obs       = static_cast<uint8_t>(config_.obs_cost);

    if (start_idx == goal_idx) return {};
    if (costmap.cells[start_idx] >= obs || costmap.cells[goal_idx] >= obs) return {};

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

    if (!found) return {};

    std::vector<std::pair<float,float>> path;
    for (int idx = goal_idx; idx != -1; idx = parent[idx]) {
        path.push_back(cell_to_world(costmap, idx % W, idx / W));
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace route_planner::astar
