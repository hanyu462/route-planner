// [TE-3] OccupancyGrid 확인 + RViz2 시각화
//
// 목적:
//   PointCloudProcessedBuffer에서 PointCloudXYZFrame을 읽어 OccupancyGridBuilder로
//   2D Occupancy Grid를 생성, OccupancyGridBuffer에 저장, ROS2 토픽 발행, 콘솔 출력
//
// 실행:
//   ros2 run route_planner te_3_node
//   --ros-args --params-file config/pointcloud2_adapter.yaml
//   -p processor_config:=config/pointcloud_processor.yaml
//   -p grid_config:=config/occupancy_grid_builder.yaml
//
// RViz2:
//   ros2 run rviz2 rviz2
//   Fixed Frame: utlidar_lidar
//   Add → Map → Topic: /route_planner/occupancy_grid
//
// 출력:
//   grid 크기, 해상도, occupied 셀 수

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/ros2/pointcloud2_adapter_node.hpp"
#include "route_planner/pointcloud/pointcloud_processor.hpp"
#include "route_planner/occupancy_grid/occupancy_grid_builder.hpp"
#include "route_planner/config/pointcloud_processor_yaml.hpp"
#include "route_planner/config/occupancy_grid_yaml.hpp"

using route_planner::pointcloud::PointCloudProcessedBuffer;
using route_planner::occupancy_grid::OccupancyGridBuffer;

static void print_grid(const route_planner::occupancy_grid::OccupancyGrid& grid)
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
}

static nav_msgs::msg::OccupancyGrid to_ros_msg(
    const route_planner::occupancy_grid::OccupancyGrid& grid,
    rclcpp::Clock& clock)
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp    = clock.now();
    msg.header.frame_id = grid.frame_id;

    msg.info.resolution = grid.resolution;
    msg.info.width      = static_cast<uint32_t>(grid.width);
    msg.info.height     = static_cast<uint32_t>(grid.height);
    msg.info.origin.position.x = static_cast<double>(grid.origin_x);
    msg.info.origin.position.y = static_cast<double>(grid.origin_y);
    msg.info.origin.position.z = 0.0;

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

    auto raw_buffer       = std::make_shared<PointCloudRawBuffer>();
    auto processed_buffer = std::make_shared<PointCloudProcessedBuffer>();
    auto grid_buffer      = std::make_shared<OccupancyGridBuffer>();

    auto adapter = std::make_shared<PointCloud2AdapterNode>(raw_buffer);
    std::thread spin_thread([&adapter]() { rclcpp::spin(adapter); });

    uint64_t last_proc_seq = 0;
    while (rclcpp::ok()) {
        if (auto snap = raw_buffer->read_if_new(last_proc_seq)) {
            last_proc_seq = snap->sequence;

            const auto processed = processor.process(*snap->value);
            processed_buffer->write(processed);

            route_planner::common::PoseXY origin_pose{};
            origin_pose.frame_id = snap->value->frame_id;
            origin_pose.qw = 1.0f;
            const auto grid = builder.build(processed, origin_pose);
            grid_buffer->write(grid);

            pub->publish(to_ros_msg(grid, *node->get_clock()));
            print_grid(grid);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
