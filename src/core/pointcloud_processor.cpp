#include "route_planner/core/pointcloud_processor.hpp"
#include "route_planner/core/pointcloud_processor_config.hpp"
#include "route_planner/core/pointcloud_transform.hpp"
#include "route_planner/core/pointcloud_filter.hpp"

#include <stdexcept>
#include <utility>

namespace route_planner::core {

PointCloudProcessor::PointCloudProcessor(PointCloudProcessorConfig config)
    : config_(std::move(config))
{
    validate_processor_config(config_);
}

PointCloudXYZFrame PointCloudProcessor::process(
    const PointCloudXYZFrame& frame) const
{
    if (frame.xyz.size() % 3 != 0) {
        throw std::invalid_argument(
            "PointCloudXYZFrame xyz size must be divisible by 3");
    }

    PointCloudXYZFrame result;
    result.stamp_ns = frame.stamp_ns;
    result.frame_id = frame.frame_id;
    result.xyz.reserve(frame.xyz.size());

    for (std::size_t i = 0; i < frame.xyz.size(); i += 3) {
        const PointXYZ raw{
            frame.xyz[i],
            frame.xyz[i + 1],
            frame.xyz[i + 2]
        };

        const PointXYZ p = config_.transform.enabled
            ? transform_point(raw, config_.transform)
            : raw;

        if (config_.filter.enabled && !accept_point(p, config_.filter)) {
            continue;
        }

        result.xyz.insert(result.xyz.end(), {p.x, p.y, p.z});
    }

    return result;
}

}  // namespace route_planner::core
