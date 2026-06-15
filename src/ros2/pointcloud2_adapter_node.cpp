#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/ros2/pointcloud2_adapter.hpp"
#include "route_planner/core/latest_buffer.hpp"
#include "route_planner/core/point_cloud_xyz_frame.hpp"

class PointCloud2AdapterNode : public rclcpp::Node {
public:
    PointCloud2AdapterNode()
        : Node("pointcloud2_adapter_node")
    {
        const std::string input_topic =
            this->declare_parameter<std::string>("input_topic");

        options_.remove_invalid_points =
            this->declare_parameter<bool>("remove_invalid_points");

        if (input_topic.empty()) {
            throw std::invalid_argument(
                "Parameter 'input_topic' must not be empty"
            );
        }

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->on_pointcloud(msg);
            }
        );

    }

private:
    void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto frame =
            route_planner::ros2::convert_pointcloud2_to_xyz_frame(
                *msg,
                options_
            );

        latest_frame_buffer_.write(std::move(frame));
    }

    route_planner::ros2::PointCloud2AdapterOptions options_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    route_planner::core::LatestBuffer<route_planner::core::PointCloudXYZFrame> latest_frame_buffer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<PointCloud2AdapterNode>();
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
