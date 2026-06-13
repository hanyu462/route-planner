#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/core/point_cloud_xyz_frame.hpp"

namespace route_planner::ros2 {

struct PointCloud2AdapterOptions {
    bool remove_invalid_points = true;
};

route_planner::core::PointCloudXYZFrame convert_pointcloud2_to_xyz_frame(
    const sensor_msgs::msg::PointCloud2& msg,
    const PointCloud2AdapterOptions& options = PointCloud2AdapterOptions{}
);

}  // namespace route_planner::ros2
