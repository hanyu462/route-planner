#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/common/pointcloud_xyz_frame.hpp"

namespace route_planner::ros2 {

struct PointCloud2AdapterOptions {
    bool remove_invalid_points = true;
};

route_planner::common::PointCloudXYZFrame convert_pointcloud2_to_xyz_frame(
    const sensor_msgs::msg::PointCloud2& msg,
    const PointCloud2AdapterOptions& options = PointCloud2AdapterOptions{}
);

sensor_msgs::msg::PointCloud2 convert_xyz_frame_to_pointcloud2(
    const route_planner::common::PointCloudXYZFrame& frame
);

}  // namespace route_planner::ros2
