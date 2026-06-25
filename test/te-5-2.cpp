// [TE-5-2] A* 경로 계획 및 RViz2 시각화
//
// 목적:
//   te-5-1 파이프라인에 AStarPlanner를 추가하여 Costmap 기반 A* 경로를 생성하고
//   nav_msgs/Path로 퍼블리시한다.
//   goal은 launch parameter (goal_x, goal_y) 로 map frame 좌표를 지정한다.
//
// 실행:
//   ros2 run route_planner te_5_2_node \
//     --ros-args \
//     --params-file config/pointcloud2_adapter.yaml \
//     --params-file config/pose_adapter.yaml \
//     -p processor_config:=config/pointcloud_processor.yaml \
//     -p grid_config:=config/occupancy_grid_builder.yaml \
//     -p costmap_config:=config/costmap_builder.yaml \
//     -p astar_config:=config/astar.yaml \
//     -p goal_x:=5.25 \
//     -p goal_y:=0.646
//
// RViz2:
//   Fixed Frame: map
//   Add → Map  → /route_planner/occupancy_grid
//   Add → Map  → /route_planner/costmap
//   Add → Pose → /route_planner/pose
//   Add → Path → /route_planner/path

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <optional>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/common/pose_xy.hpp"
#include "route_planner/ros2/pointcloud2_adapter_node.hpp"
#include "route_planner/ros2/pose_adapter_node.hpp"
#include "route_planner/ros2/pose_adapter.hpp"
#include "route_planner/pointcloud/pointcloud_processor.hpp"
#include "route_planner/occupancy_grid/occupancy_grid_builder.hpp"
#include "route_planner/costmap/costmap_builder.hpp"
#include "route_planner/astar/astar_planner.hpp"
#include "route_planner/config/pointcloud_processor_yaml.hpp"
#include "route_planner/config/occupancy_grid_yaml.hpp"
#include "route_planner/config/costmap_yaml.hpp"
#include "route_planner/config/astar_yaml.hpp"

using route_planner::pointcloud::PointCloudProcessedBuffer;
using route_planner::occupancy_grid::OccupancyGridBuffer;

// ── helpers ──────────────────────────────────────────────────────────────────

static nav_msgs::msg::OccupancyGrid grid_to_ros_msg(
    const route_planner::occupancy_grid::OccupancyGrid& grid,
    rclcpp::Clock& clock)
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp              = clock.now();
    msg.header.frame_id           = grid.frame_id;
    msg.info.resolution           = grid.resolution;
    msg.info.width                = static_cast<uint32_t>(grid.width);
    msg.info.height               = static_cast<uint32_t>(grid.height);
    msg.info.origin.position.x    = static_cast<double>(grid.origin_x);
    msg.info.origin.position.y    = static_cast<double>(grid.origin_y);
    msg.info.origin.position.z    = 0.0;
    msg.info.origin.orientation.w = 1.0;
    msg.data = grid.cells;
    return msg;
}

static nav_msgs::msg::OccupancyGrid costmap_to_ros_msg(
    const route_planner::costmap::Costmap& costmap,
    rclcpp::Clock& clock,
    uint8_t lethal_cost,
    uint8_t max_soft_cost)
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp              = clock.now();
    msg.header.frame_id           = costmap.frame_id;
    msg.info.resolution           = costmap.resolution;
    msg.info.width                = static_cast<uint32_t>(costmap.width);
    msg.info.height               = static_cast<uint32_t>(costmap.height);
    msg.info.origin.position.x    = static_cast<double>(costmap.origin_x);
    msg.info.origin.position.y    = static_cast<double>(costmap.origin_y);
    msg.info.origin.position.z    = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(costmap.cells.size());
    for (std::size_t i = 0; i < costmap.cells.size(); ++i) {
        const uint8_t c = costmap.cells[i];
        if (c >= lethal_cost) {
            msg.data[i] = int8_t{100};
        } else if (c == 0) {
            msg.data[i] = int8_t{0};
        } else {
            msg.data[i] = static_cast<int8_t>(
                1 + static_cast<int>(c) * 98 / static_cast<int>(max_soft_cost));
        }
    }

    return msg;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("astar_node");

    const std::string processor_config_path =
        node->declare_parameter<std::string>("processor_config",
                                             "config/pointcloud_processor.yaml");
    const std::string grid_config_path =
        node->declare_parameter<std::string>("grid_config",
                                             "config/occupancy_grid_builder.yaml");
    const std::string costmap_config_path =
        node->declare_parameter<std::string>("costmap_config",
                                             "config/costmap_builder.yaml");
    const std::string astar_config_path =
        node->declare_parameter<std::string>("astar_config",
                                             "config/astar.yaml");
    const float goal_x = node->declare_parameter<float>("goal_x", 2.0f);
    const float goal_y = node->declare_parameter<float>("goal_y", 0.0f);

    const auto processor_config =
        route_planner::config::load_processor_config(processor_config_path);
    const auto grid_config =
        route_planner::config::load_occupancy_grid_config(grid_config_path);
    const auto costmap_config =
        route_planner::config::load_costmap_config(costmap_config_path);
    const auto astar_config =
        route_planner::config::load_astar_config(astar_config_path);

    route_planner::pointcloud::PointCloudProcessor processor{processor_config};
    route_planner::occupancy_grid::OccupancyGridBuilder grid_builder{grid_config};
    route_planner::costmap::CostmapBuilder costmap_builder{costmap_config};
    route_planner::astar::AStarPlanner planner{astar_config};

    auto grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/route_planner/occupancy_grid", rclcpp::SensorDataQoS());
    auto costmap_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/route_planner/costmap", rclcpp::SensorDataQoS());
    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/route_planner/pose", rclcpp::SensorDataQoS());
    auto path_pub = node->create_publisher<nav_msgs::msg::Path>(
        "/route_planner/path", rclcpp::SensorDataQoS());

    // goal: map frame (x, y), set via ROS2 parameters at launch
    const std::pair<float,float> current_goal = {goal_x, goal_y};
    std::printf("[A*] goal set: (%.2f, %.2f)\n", goal_x, goal_y);

    auto raw_buffer       = std::make_shared<PointCloudRawBuffer>();
    auto processed_buffer = std::make_shared<PointCloudProcessedBuffer>();
    auto grid_buffer      = std::make_shared<OccupancyGridBuffer>();

    auto adapter = std::make_shared<PointCloud2AdapterNode>(raw_buffer);
    std::thread spin_thread([&adapter]() { rclcpp::spin(adapter); });

    std::shared_ptr<PoseBuffer>      pos_buffer;
    std::shared_ptr<PoseAdapterNode> uwb_node;
    std::thread                      uwb_spin_thread;

    pos_buffer      = std::make_shared<PoseBuffer>();
    uwb_node        = std::make_shared<PoseAdapterNode>(pos_buffer);
    uwb_spin_thread = std::thread([&uwb_node]() { rclcpp::spin(uwb_node); });

    uint64_t last_proc_seq = 0;
    while (rclcpp::ok()) {
        if (auto snap = raw_buffer->read_if_new(last_proc_seq)) {
            last_proc_seq = snap->sequence;

            std::optional<route_planner::common::PoseXY> current_pose;
            if (auto pos = pos_buffer->read()) {
                current_pose = *pos->value;
                pose_pub->publish(
                    route_planner::ros2::convert_pose_xy_to_pose_stamped(*current_pose));
            }

            const auto processed = processor.process(*snap->value);
            processed_buffer->write(processed);

            if (current_pose) {
                const auto grid = grid_builder.build(processed, *current_pose);
                grid_buffer->write(grid);

                const auto costmap = costmap_builder.build(grid);

                grid_pub->publish(grid_to_ros_msg(grid, *node->get_clock()));
                costmap_pub->publish(costmap_to_ros_msg(
                    costmap, *node->get_clock(),
                    costmap_config.lethal_cost, costmap_config.max_soft_cost));

                // ── A* path planning ─────────────────────────────────────────────

                {
                    const auto result = planner.plan(
                        costmap,
                        {current_pose->x, current_pose->y},
                        current_goal);

                    nav_msgs::msg::Path path_msg;
                    path_msg.header.stamp    = node->get_clock()->now();
                    path_msg.header.frame_id = result.frame_id;

                    for (const auto& [x, y] : result.waypoints) {
                        geometry_msgs::msg::PoseStamped ps;
                        ps.header = path_msg.header;
                        ps.pose.position.x    = static_cast<double>(x);
                        ps.pose.position.y    = static_cast<double>(y);
                        ps.pose.orientation.w = 1.0;
                        path_msg.poses.push_back(ps);
                    }

                    path_pub->publish(path_msg);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spin_thread.join();
    if (uwb_spin_thread.joinable()) uwb_spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
