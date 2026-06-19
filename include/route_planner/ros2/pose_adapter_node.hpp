#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "route_planner/common/latest_buffer.hpp"
#include "route_planner/common/pose_xy.hpp"
#include "route_planner/ros2/pose_adapter.hpp"

using PoseBuffer = route_planner::common::LatestBuffer<route_planner::common::PoseXY>;

class PoseAdapterNode : public rclcpp::Node {
public:
    explicit PoseAdapterNode(std::shared_ptr<PoseBuffer> buffer)
        : Node("pose_adapter_node")
        , buffer_(std::move(buffer))
    {
        const std::string input_topic =
            this->declare_parameter<std::string>("input_topic");

        if (input_topic.empty()) {
            throw std::invalid_argument("Parameter 'input_topic' must not be empty");
        }

        sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            input_topic,
            rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->on_pose(msg);
            }
        );
    }

private:
    void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        buffer_->write(
            route_planner::ros2::convert_pose_stamped_to_pose_xy(*msg));
    }

    std::shared_ptr<PoseBuffer> buffer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};
