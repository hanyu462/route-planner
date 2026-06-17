#include "route_planner/pointcloud/pointcloud_processor_config.hpp"

#include <stdexcept>

namespace route_planner::pointcloud {

void validate_processor_config(const PointCloudProcessorConfig& config)
{
    const auto& f = config.filter;

    if (f.min_x > f.max_x)
        throw std::invalid_argument("filter.min_x must not exceed filter.max_x");

    if (f.min_y > f.max_y)
        throw std::invalid_argument("filter.min_y must not exceed filter.max_y");

    if (f.min_z > f.max_z)
        throw std::invalid_argument("filter.min_z must not exceed filter.max_z");
}

}  // namespace route_planner::pointcloud
