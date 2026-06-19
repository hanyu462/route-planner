#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "route_planner/ros2/pose_adapter_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto buffer = std::make_shared<PoseBuffer>();
        auto node   = std::make_shared<PoseAdapterNode>(buffer);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(
            rclcpp::get_logger("pose_adapter_node"),
            "Failed to start node: %s",
            e.what()
        );
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
