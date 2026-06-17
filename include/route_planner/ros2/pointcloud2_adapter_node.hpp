#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/ros2/pointcloud2_adapter.hpp"
#include "route_planner/common/latest_buffer.hpp"
#include "route_planner/common/pointcloud_xyz_frame.hpp"

using FrameBuffer = route_planner::common::LatestBuffer<route_planner::common::PointCloudXYZFrame>;

class PointCloud2AdapterNode : public rclcpp::Node {
public:
    explicit PointCloud2AdapterNode(std::shared_ptr<FrameBuffer> buffer)
        : Node("pointcloud2_adapter_node")
        , buffer_(std::move(buffer))
    {
        const std::string input_topic =
            this->declare_parameter<std::string>("input_topic");

        options_.remove_invalid_points =
            this->declare_parameter<bool>("remove_invalid_points", true);

        if (input_topic.empty()) {
            throw std::invalid_argument(
                "Parameter 'input_topic' must not be empty");
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
        buffer_->write(
            route_planner::ros2::convert_pointcloud2_to_xyz_frame(
                *msg, options_));
    }

    std::shared_ptr<FrameBuffer> buffer_;
    route_planner::ros2::PointCloud2AdapterOptions options_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};
