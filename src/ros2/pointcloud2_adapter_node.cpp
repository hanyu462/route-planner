#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "route_planner/ros2/pointcloud2_adapter_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto buffer = std::make_shared<FrameBuffer>();
        auto node   = std::make_shared<PointCloud2AdapterNode>(buffer);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(
            rclcpp::get_logger("pointcloud2_adapter_node"),
            "Failed to start node: %s",
            e.what()
        );
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
