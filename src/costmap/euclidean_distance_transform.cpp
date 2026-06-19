#include "route_planner/costmap/euclidean_distance_transform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace route_planner::costmap {

// Large finite sentinel: represents "no obstacle in this row/column".
// Must be large enough so that (sentinel + max_dist²) does not overflow float.
static constexpr float kInf = 1e18f;

// 1-D distance transform (Felzenszwalb & Huttenlocher, 2012).
// Computes d[i] = min_j { (i-j)^2 + f[j] } for i in [0, n).
// v and z are caller-provided scratch buffers of size n+1 and n+2.
static void dt1d(const float* f, float* d, int* v, float* z, int n)
{
    int k = 0;
    v[0]   = 0;
    z[0]   = -std::numeric_limits<float>::infinity();
    z[1]   =  std::numeric_limits<float>::infinity();

    for (int q = 1; q < n; ++q) {
        const float fq = f[q];
        float s;
        while (true) {
            const int   vk = v[k];
            s = ((fq + static_cast<float>(q * q))
                 - (f[vk] + static_cast<float>(vk * vk)))
                / (2.0f * static_cast<float>(q - vk));
            if (s > z[k]) break;
            --k;
        }
        ++k;
        v[k]     = q;
        z[k]     = s;
        z[k + 1] = std::numeric_limits<float>::infinity();
    }

    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < static_cast<float>(q)) ++k;
        const float diff = static_cast<float>(q - v[k]);
        d[q] = diff * diff + f[v[k]];
    }
}

std::vector<float> compute_edt_sq(
    const std::vector<int8_t>& cells,
    int width,
    int height,
    int8_t occupied_value)
{
    const int N    = width * height;
    const int maxd = std::max(width, height);

    std::vector<float> row_buf(static_cast<std::size_t>(maxd));
    std::vector<float> out_buf(static_cast<std::size_t>(maxd));
    std::vector<int>   v_buf(static_cast<std::size_t>(maxd + 1));
    std::vector<float> z_buf(static_cast<std::size_t>(maxd + 2));

    // Phase 1: 1-D EDT along each row → squared horizontal distance per cell.
    std::vector<float> d1(static_cast<std::size_t>(N));

    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            row_buf[static_cast<std::size_t>(c)] =
                (cells[static_cast<std::size_t>(r * width + c)] == occupied_value)
                    ? 0.0f : kInf;
        }
        dt1d(row_buf.data(), out_buf.data(), v_buf.data(), z_buf.data(), width);
        for (int c = 0; c < width; ++c) {
            d1[static_cast<std::size_t>(r * width + c)] =
                out_buf[static_cast<std::size_t>(c)];
        }
    }

    // Phase 2: 1-D EDT along each column using d1 → exact 2-D squared distance.
    std::vector<float> dist_sq(static_cast<std::size_t>(N));

    for (int c = 0; c < width; ++c) {
        for (int r = 0; r < height; ++r) {
            row_buf[static_cast<std::size_t>(r)] =
                d1[static_cast<std::size_t>(r * width + c)];
        }
        dt1d(row_buf.data(), out_buf.data(), v_buf.data(), z_buf.data(), height);
        for (int r = 0; r < height; ++r) {
            dist_sq[static_cast<std::size_t>(r * width + c)] =
                out_buf[static_cast<std::size_t>(r)];
        }
    }

    return dist_sq;
}

}  // namespace route_planner::costmap
