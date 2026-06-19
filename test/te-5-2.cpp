// [TE-5-2] A* 경로 계획 및 RViz2 시각화
//
// 목적:
//   te-5-1 파이프라인에 AStarPlanner를 추가하여 Costmap 기반 A* 경로를 생성하고
//   nav_msgs/Path로 퍼블리시한다. goal은 config/astar.yaml에서 map frame으로 설정.
//
// 실행:
//   ros2 run route_planner te_5_2_node
//   --ros-args --params-file config/pointcloud2_adapter.yaml
//             --params-file config/pose_adapter.yaml
//   -p processor_config:=config/pointcloud_processor.yaml
//   -p grid_config:=config/occupancy_grid_builder.yaml
//   -p costmap_config:=config/costmap_builder.yaml
//   -p astar_config:=config/astar.yaml
//
// RViz2:
//   Fixed Frame: map
//   Add → Map  → /route_planner/occupancy_grid
//   Add → Map  → /route_planner/costmap
//   Add → Pose → /route_planner/pose
//   Add → Path → /route_planner/path

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <thread>

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

static float extract_yaw(const route_planner::common::PoseXY& pose)
{
    return std::atan2(
        2.0f * (pose.qw * pose.qz + pose.qx * pose.qy),
        1.0f - 2.0f * (pose.qy * pose.qy + pose.qz * pose.qz));
}

static nav_msgs::msg::OccupancyGrid grid_to_ros_msg(
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
        const float yaw = extract_yaw(*pose);
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

static nav_msgs::msg::OccupancyGrid costmap_to_ros_msg(
    const route_planner::costmap::Costmap& costmap,
    rclcpp::Clock& clock,
    const route_planner::common::PoseXY* pose,
    uint8_t lethal_cost,
    uint8_t max_soft_cost)
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp    = clock.now();
    msg.info.resolution = costmap.resolution;
    msg.info.width      = static_cast<uint32_t>(costmap.width);
    msg.info.height     = static_cast<uint32_t>(costmap.height);
    msg.info.origin.position.z = 0.0;

    if (pose) {
        const float yaw = extract_yaw(*pose);
        msg.header.frame_id = pose->frame_id;
        msg.info.origin.position.x = static_cast<double>(
            pose->x + std::cos(yaw) * costmap.origin_x - std::sin(yaw) * costmap.origin_y);
        msg.info.origin.position.y = static_cast<double>(
            pose->y + std::sin(yaw) * costmap.origin_x + std::cos(yaw) * costmap.origin_y);
        msg.info.origin.orientation.z = static_cast<double>(std::sin(yaw / 2.0f));
        msg.info.origin.orientation.w = static_cast<double>(std::cos(yaw / 2.0f));
    } else {
        msg.header.frame_id = costmap.frame_id;
        msg.info.origin.position.x = static_cast<double>(costmap.origin_x);
        msg.info.origin.position.y = static_cast<double>(costmap.origin_y);
        msg.info.origin.orientation.w = 1.0;
    }

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

    auto raw_buffer       = std::make_shared<PointCloudRawBuffer>();
    auto processed_buffer = std::make_shared<PointCloudProcessedBuffer>();
    auto grid_buffer      = std::make_shared<OccupancyGridBuffer>();

    auto adapter = std::make_shared<PointCloud2AdapterNode>(raw_buffer);
    std::thread spin_thread([&adapter]() { rclcpp::spin(adapter); });

    std::shared_ptr<PoseBuffer>      pos_buffer;
    std::shared_ptr<PoseAdapterNode> uwb_node;
    std::thread                      uwb_spin_thread;

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
                    pose_ptr     = &current_pose.value();
                    pose_pub->publish(
                        route_planner::ros2::convert_pose_xy_to_pose_stamped(*current_pose));
                }
            }

            const auto processed = processor.process(*snap->value);
            processed_buffer->write(processed);

            const auto grid = grid_builder.build(processed);
            grid_buffer->write(grid);

            const auto costmap = costmap_builder.build(grid);

            grid_pub->publish(grid_to_ros_msg(grid, *node->get_clock(), pose_ptr));
            costmap_pub->publish(costmap_to_ros_msg(
                costmap, *node->get_clock(), pose_ptr,
                costmap_config.lethal_cost, costmap_config.max_soft_cost));

            // ── A* path planning ─────────────────────────────────────────────

            // Convert map-frame goal to robot-local (costmap) frame
            float goal_local_x = astar_config.goal_x;
            float goal_local_y = astar_config.goal_y;
            std::string path_frame = grid.frame_id;

            if (pose_ptr) {
                const float yaw = extract_yaw(*pose_ptr);
                const float dx  = astar_config.goal_x - pose_ptr->x;
                const float dy  = astar_config.goal_y - pose_ptr->y;
                goal_local_x    =  std::cos(yaw) * dx + std::sin(yaw) * dy;
                goal_local_y    = -std::sin(yaw) * dx + std::cos(yaw) * dy;
                path_frame      = pose_ptr->frame_id;
            }

            const auto path = planner.plan(costmap, 0.0f, 0.0f, goal_local_x, goal_local_y);

            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp    = node->get_clock()->now();
            path_msg.header.frame_id = path_frame;

            if (path.empty()) {
                std::printf("[A*] No path found (goal_local: %.2f, %.2f)\n",
                            goal_local_x, goal_local_y);
            } else {
                for (const auto& [lx, ly] : path) {
                    geometry_msgs::msg::PoseStamped ps;
                    ps.header = path_msg.header;
                    ps.pose.orientation.w = 1.0;

                    if (pose_ptr) {
                        // Convert robot-local waypoint back to map frame
                        const float yaw = extract_yaw(*pose_ptr);
                        ps.pose.position.x = static_cast<double>(
                            pose_ptr->x + std::cos(yaw) * lx - std::sin(yaw) * ly);
                        ps.pose.position.y = static_cast<double>(
                            pose_ptr->y + std::sin(yaw) * lx + std::cos(yaw) * ly);
                    } else {
                        ps.pose.position.x = static_cast<double>(lx);
                        ps.pose.position.y = static_cast<double>(ly);
                    }
                    path_msg.poses.push_back(ps);
                }

                const auto& first = path_msg.poses.front().pose.position;
                const auto& last  = path_msg.poses.back().pose.position;
                std::printf("[A*] path: %zu waypoints  "
                            "start=(%.2f, %.2f)  goal=(%.2f, %.2f)\n",
                            path_msg.poses.size(),
                            first.x, first.y, last.x, last.y);
            }

            path_pub->publish(path_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spin_thread.join();
    if (uwb_spin_thread.joinable()) uwb_spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
