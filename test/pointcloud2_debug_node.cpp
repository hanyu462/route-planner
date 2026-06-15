#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/ros2/pointcloud2_adapter.hpp"

class PointCloud2DebugNode : public rclcpp::Node {
public:
    PointCloud2DebugNode()
        : Node("pointcloud2_debug_node")
    {
        const std::string input_topic =
            this->declare_parameter<std::string>("input_topic");

        options_.remove_invalid_points =
            this->declare_parameter<bool>("remove_invalid_points", true);

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
        auto frame = route_planner::ros2::convert_pointcloud2_to_xyz_frame(
            *msg, options_);

        std::printf("\n── PointCloudXYZFrame ──────────────────\n");
        std::printf("  stamp_ns  : %ld\n", frame.stamp_ns);
        std::printf("  frame_id  : %s\n",  frame.frame_id.c_str());
        std::printf("  points    : %zu\n", frame.point_count());

        constexpr std::size_t PREVIEW = 5;
        const std::size_t n = std::min(frame.point_count(), PREVIEW);
        for (std::size_t i = 0; i < n; ++i) {
            std::printf("  [%zu] x=%.4f  y=%.4f  z=%.4f\n",
                i,
                frame.xyz[i * 3 + 0],
                frame.xyz[i * 3 + 1],
                frame.xyz[i * 3 + 2]);
        }
        if (frame.point_count() > PREVIEW) {
            std::printf("  ... (%zu more)\n", frame.point_count() - PREVIEW);
        }
    }

    route_planner::ros2::PointCloud2AdapterOptions options_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloud2DebugNode>());
    rclcpp::shutdown();
    return 0;
}
