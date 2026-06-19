#include "route_planner/ros2/pose_adapter.hpp"
#include "route_planner/ros2/stamp_utils.hpp"

namespace route_planner::ros2 {

route_planner::common::PoseXY convert_pose_stamped_to_pose_xy(
    const geometry_msgs::msg::PoseStamped& msg)
{
    return {
        to_nanoseconds(msg.header.stamp),
        msg.header.frame_id,
        static_cast<float>(msg.pose.position.x),
        static_cast<float>(msg.pose.position.y),
        static_cast<float>(msg.pose.orientation.x),
        static_cast<float>(msg.pose.orientation.y),
        static_cast<float>(msg.pose.orientation.z),
        static_cast<float>(msg.pose.orientation.w),
    };
}

geometry_msgs::msg::PoseStamped convert_pose_xy_to_pose_stamped(
    const route_planner::common::PoseXY& pose)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp.sec     = static_cast<int32_t>(pose.stamp_ns / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(pose.stamp_ns % 1'000'000'000LL);
    msg.header.frame_id      = pose.frame_id;
    msg.pose.position.x      = static_cast<double>(pose.x);
    msg.pose.position.y      = static_cast<double>(pose.y);
    msg.pose.position.z      = 0.0;
    msg.pose.orientation.x   = static_cast<double>(pose.qx);
    msg.pose.orientation.y   = static_cast<double>(pose.qy);
    msg.pose.orientation.z   = static_cast<double>(pose.qz);
    msg.pose.orientation.w   = static_cast<double>(pose.qw);
    return msg;
}

}  // namespace route_planner::ros2
