// [TE-4] 로봇 위치 offset 적용 OccupancyGrid 확인 + RViz2 시각화
//
// 목적:
//   TE-3에 PoseAdapterNode를 추가하여 로봇 위치를 grid origin에 반영.
//   align_to_pose_frame: true 시 로봇 위치를 읽어 builder.build()에 전달.
//
// 실행:
//   ros2 run route_planner te_4_node
//   --ros-args --params-file config/pointcloud2_adapter.yaml
//             --params-file config/pose_adapter.yaml
//   -p processor_config:=config/pointcloud_processor.yaml
//   -p grid_config:=config/occupancy_grid_builder.yaml
//
// RViz2:
//   ros2 run rviz2 rviz2
//   Fixed Frame: utlidar_lidar
//   Add → Map   → Topic: /route_planner/occupancy_grid
//   Add → Pose  → Topic: /route_planner/pose  (align_to_pose_frame: true 시)
//
// 출력:
//   grid 크기, 해상도, occupied 셀 수, 로봇 위치/방향 (align_to_pose_frame: true 시)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/common/pose_xy.hpp"
#include "route_planner/ros2/pointcloud2_adapter_node.hpp"
#include "route_planner/ros2/pose_adapter_node.hpp"
#include "route_planner/ros2/pose_adapter.hpp"
#include "route_planner/pointcloud/pointcloud_processor.hpp"
#include "route_planner/occupancy_grid/occupancy_grid_builder.hpp"
#include "route_planner/config/pointcloud_processor_yaml.hpp"
#include "route_planner/config/occupancy_grid_yaml.hpp"

using route_planner::pointcloud::PointCloudProcessedBuffer;
using route_planner::occupancy_grid::OccupancyGridBuffer;

static void print_grid(const route_planner::occupancy_grid::OccupancyGrid& grid,
                       const route_planner::common::PoseXY* pose)
{
    const int occupied = static_cast<int>(
        std::count(grid.cells.begin(), grid.cells.end(), int8_t{100}));

    std::printf("\n── OccupancyGrid ────────────────────────\n");
    std::printf("  stamp_ns : %ld\n", grid.stamp_ns);
    std::printf("  frame_id : %s\n",  grid.frame_id.c_str());
    std::printf("  size     : %d x %d  (%.2f x %.2f m)\n",
        grid.width, grid.height,
        grid.width  * grid.resolution,
        grid.height * grid.resolution);
    std::printf("  occupied : %d / %d cells\n", occupied, grid.width * grid.height);
    if (pose) {
        const float yaw = std::atan2(
            2.0f * (pose->qw * pose->qz + pose->qx * pose->qy),
            1.0f - 2.0f * (pose->qy * pose->qy + pose->qz * pose->qz));
        std::printf("── Location ─────────────────────────────\n");
        std::printf("  frame_id : %s\n",  pose->frame_id.c_str());
        std::printf("  position : x=%.3f  y=%.3f\n", pose->x, pose->y);
        std::printf("  orient   : qx=%.3f  qy=%.3f  qz=%.3f  qw=%.3f\n",
            pose->qx, pose->qy, pose->qz, pose->qw);
        std::printf("  yaw      : %.2f deg\n", yaw * 180.0f / M_PI);
    }
}

static nav_msgs::msg::OccupancyGrid to_ros_msg(
    const route_planner::occupancy_grid::OccupancyGrid& grid,
    rclcpp::Clock& clock,
    const route_planner::common::PoseXY* pose)
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp    = clock.now();
    msg.info.resolution = grid.resolution;
    msg.info.width      = static_cast<uint32_t>(grid.width);
    msg.info.height     = static_cast<uint32_t>(grid.height);
    msg.info.origin.position.z = 0.0;

    if (pose) {
        const float yaw = std::atan2(
            2.0f * (pose->qw * pose->qz + pose->qx * pose->qy),
            1.0f - 2.0f * (pose->qy * pose->qy + pose->qz * pose->qz));

        msg.header.frame_id = pose->frame_id;
        msg.info.origin.position.x = static_cast<double>(
            pose->x + std::cos(yaw) * grid.origin_x - std::sin(yaw) * grid.origin_y);
        msg.info.origin.position.y = static_cast<double>(
            pose->y + std::sin(yaw) * grid.origin_x + std::cos(yaw) * grid.origin_y);
        msg.info.origin.orientation.z = static_cast<double>(std::sin(yaw / 2.0f));
        msg.info.origin.orientation.w = static_cast<double>(std::cos(yaw / 2.0f));
    } else {
        msg.header.frame_id = grid.frame_id;
        msg.info.origin.position.x = static_cast<double>(grid.origin_x);
        msg.info.origin.position.y = static_cast<double>(grid.origin_y);
        msg.info.origin.orientation.w = 1.0;
    }

    msg.data = grid.cells;
    return msg;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("occupancy_grid_node");

    const std::string processor_config_path =
        node->declare_parameter<std::string>("processor_config",
                                             "config/pointcloud_processor.yaml");
    const std::string grid_config_path =
        node->declare_parameter<std::string>("grid_config",
                                             "config/occupancy_grid_builder.yaml");

    const auto processor_config =
        route_planner::config::load_processor_config(processor_config_path);
    const auto grid_config =
        route_planner::config::load_occupancy_grid_config(grid_config_path);

    route_planner::pointcloud::PointCloudProcessor processor{processor_config};
    route_planner::occupancy_grid::OccupancyGridBuilder builder{grid_config};

    auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/route_planner/occupancy_grid", rclcpp::SensorDataQoS());
    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/route_planner/pose", rclcpp::SensorDataQoS());

    auto raw_buffer       = std::make_shared<PointCloudRawBuffer>();
    auto processed_buffer = std::make_shared<PointCloudProcessedBuffer>();
    auto grid_buffer      = std::make_shared<OccupancyGridBuffer>();

    auto adapter = std::make_shared<PointCloud2AdapterNode>(raw_buffer);
    std::thread spin_thread([&adapter]() { rclcpp::spin(adapter); });

    std::shared_ptr<PoseBuffer> pos_buffer;
    std::shared_ptr<PoseAdapterNode> uwb_node;
    std::thread                     uwb_spin_thread;

    if (grid_config.align_to_pose_frame) {
        pos_buffer      = std::make_shared<PoseBuffer>();
        uwb_node        = std::make_shared<PoseAdapterNode>(pos_buffer);
        uwb_spin_thread = std::thread([&uwb_node]() { rclcpp::spin(uwb_node); });
    }

    uint64_t last_proc_seq = 0;
    while (rclcpp::ok()) {
        if (auto snap = raw_buffer->read_if_new(last_proc_seq)) {
            last_proc_seq = snap->sequence;

            const route_planner::common::PoseXY* pose_ptr = nullptr;
            std::optional<route_planner::common::PoseXY> current_pose;
            if (grid_config.align_to_pose_frame && pos_buffer) {
                if (auto pos = pos_buffer->read()) {
                    current_pose = *pos->value;
                    pose_ptr = &current_pose.value();
                    pose_pub->publish(
                        route_planner::ros2::convert_pose_xy_to_pose_stamped(*current_pose));
                }
            }

            const auto processed = processor.process(*snap->value);
            processed_buffer->write(processed);

            const auto grid = builder.build(processed);
            grid_buffer->write(grid);

            pub->publish(to_ros_msg(grid, *node->get_clock(), pose_ptr));
            print_grid(grid, pose_ptr);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spin_thread.join();
    if (uwb_spin_thread.joinable()) uwb_spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
