#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/ros2/pointcloud2_adapter.hpp"

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

        RCLCPP_INFO(
            this->get_logger(),
            "pointcloud2_adapter_node ready. input_topic=%s remove_invalid_points=%s",
            input_topic.c_str(),
            options_.remove_invalid_points ? "true" : "false"
        );
    }

private:
    void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            auto frame =
                route_planner::ros2::convert_pointcloud2_to_xyz_frame(
                    *msg,
                    options_
                );

            if (frame.empty()) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "No valid points after PointCloud2 conversion"
                );
                return;
            }

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "PointCloudXYZFrame: points=%zu frame_id=%s",
                frame.point_count(),
                frame.frame_id.c_str()
            );

            // TODO: latest_frame_buffer.write(std::move(frame));

        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "PointCloud2 conversion failed: %s",
                e.what()
            );
        }
    }

    route_planner::ros2::PointCloud2AdapterOptions options_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
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
