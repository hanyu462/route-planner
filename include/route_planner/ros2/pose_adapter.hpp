#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "route_planner/common/pose_xy.hpp"

namespace route_planner::ros2 {

route_planner::common::PoseXY convert_pose_stamped_to_pose_xy(
    const geometry_msgs::msg::PoseStamped& msg);

geometry_msgs::msg::PoseStamped convert_pose_xy_to_pose_stamped(
    const route_planner::common::PoseXY& pose);

}  // namespace route_planner::ros2
