// [TE-6] RoutePlannerPipeline 검증
//
// 목적:
//   RoutePlannerPipeline을 사용하여 T1~T6 스레드 분리 구조를 검증한다.
//   te-5-2와 동일한 출력(occupancy_grid, costmap, pose, path)을 퍼블리시한다.
//
// 실행:
//   ros2 run route_planner te_6_node
//   --ros-args --params-file config/pointcloud2_adapter.yaml
//             --params-file config/pose_adapter.yaml
//   -p processor_config:=config/pointcloud_processor.yaml
//   -p grid_config:=config/occupancy_grid_builder.yaml
//   -p costmap_config:=config/costmap_builder.yaml
//   -p astar_config:=config/astar.yaml
//   -p goal_x:=5.17 -p goal_y:=0.7
//
// RViz2:
//   Fixed Frame: map
//   Add → Map  → /route_planner/occupancy_grid
//   Add → Map  → /route_planner/costmap
//   Add → Pose → /route_planner/pose
//   Add → Path → /route_planner/path

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "route_planner/pipeline/route_planner_pipeline.hpp"
#include "route_planner/ros2/pose_adapter.hpp"
#include "route_planner/config/costmap_yaml.hpp"

using route_planner::pipeline::RoutePlannerPipeline;

// ── helpers ───────────────────────────────────────────────────────────────────

static float extract_yaw(const route_planner::common::PoseXY& p)
{
    return std::atan2(
        2.0f * (p.qw * p.qz + p.qx * p.qy),
        1.0f - 2.0f * (p.qy * p.qy + p.qz * p.qz));
}

static nav_msgs::msg::OccupancyGrid grid_to_ros(
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

static nav_msgs::msg::OccupancyGrid costmap_to_ros(
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
        if (c >= lethal_cost)       msg.data[i] = int8_t{100};
        else if (c == 0)            msg.data[i] = int8_t{0};
        else msg.data[i] = static_cast<int8_t>(
            1 + static_cast<int>(c) * 98 / static_cast<int>(max_soft_cost));
    }
    return msg;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("te_6_node");

    const std::string processor_config =
        node->declare_parameter<std::string>("processor_config",
                                             "config/pointcloud_processor.yaml");
    const std::string grid_config =
        node->declare_parameter<std::string>("grid_config",
                                             "config/occupancy_grid_builder.yaml");
    const std::string costmap_config =
        node->declare_parameter<std::string>("costmap_config",
                                             "config/costmap_builder.yaml");
    const std::string astar_config =
        node->declare_parameter<std::string>("astar_config",
                                             "config/astar.yaml");
    const std::string pipeline_config =
        node->declare_parameter<std::string>("pipeline_config",
                                             "config/pipeline.yaml");
    const float goal_x = node->declare_parameter<float>("goal_x", 2.0f);
    const float goal_y = node->declare_parameter<float>("goal_y", 0.0f);

    // ── pipeline ──────────────────────────────────────────────────────────────
    RoutePlannerPipeline pipeline({
        processor_config, grid_config, costmap_config, astar_config, pipeline_config
    });
    pipeline.set_goal(goal_x, goal_y);
    pipeline.start();

    std::printf("[te-6] goal set: (%.2f, %.2f)\n", goal_x, goal_y);

    // ── publishers ────────────────────────────────────────────────────────────
    auto grid_pub    = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/route_planner/occupancy_grid", rclcpp::SensorDataQoS());
    auto costmap_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/route_planner/costmap", rclcpp::SensorDataQoS());
    auto pose_pub    = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/route_planner/pose", rclcpp::SensorDataQoS());
    auto path_pub    = node->create_publisher<nav_msgs::msg::Path>(
        "/route_planner/path", rclcpp::SensorDataQoS());

    // Load costmap config for normalization params
    const auto cm_cfg =
        route_planner::config::load_costmap_config(costmap_config);

    // ── publish loop ──────────────────────────────────────────────────────────
    uint64_t last_grid_seq    = 0;
    uint64_t last_costmap_seq = 0;
    uint64_t last_path_seq    = 0;

    auto last_hz_print = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        // Pose
        const route_planner::common::PoseXY* pose_ptr = nullptr;
        std::optional<route_planner::common::PoseXY> current_pose;
        if (auto snap = pipeline.pose_buffer()->read()) {
            current_pose = *snap->value;
            pose_ptr     = &current_pose.value();
            pose_pub->publish(
                route_planner::ros2::convert_pose_xy_to_pose_stamped(*current_pose));
        }

        // OccupancyGrid
        if (auto snap = pipeline.occupancy_grid_buffer()->read_if_new(last_grid_seq)) {
            last_grid_seq = snap->sequence;
            grid_pub->publish(grid_to_ros(*snap->value, *node->get_clock(), pose_ptr));
        }

        // Costmap
        if (auto snap = pipeline.costmap_buffer()->read_if_new(last_costmap_seq)) {
            last_costmap_seq = snap->sequence;
            costmap_pub->publish(costmap_to_ros(
                *snap->value, *node->get_clock(), pose_ptr,
                cm_cfg.lethal_cost, cm_cfg.max_soft_cost));
        }

        // Path
        if (auto snap = pipeline.path_buffer()->read_if_new(last_path_seq)) {
            last_path_seq = snap->sequence;
            const route_planner::common::Path& path = *snap->value;

            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp    = node->get_clock()->now();
            path_msg.header.frame_id = path.frame_id;

            for (const auto& [x, y] : path.waypoints) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path_msg.header;
                ps.pose.position.x    = static_cast<double>(x);
                ps.pose.position.y    = static_cast<double>(y);
                ps.pose.orientation.w = 1.0;
                path_msg.poses.push_back(ps);
            }
            path_pub->publish(path_msg);
        }

        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(now - last_hz_print).count() >= 5.0f) {
            const auto hz = pipeline.thread_hz();
            std::printf("[Hz] T1 pointcloud2_spin  %.1f Hz\n", hz.t1_pointcloud2_spin);
            std::printf("[Hz] T2 pose_spin         %.1f Hz\n", hz.t2_pose_spin);
            std::printf("[Hz] T3 processor         %.1f Hz\n", hz.t3_processor);
            std::printf("[Hz] T4 occupancy_grid    %.1f Hz\n", hz.t4_occupancy_grid);
            std::printf("[Hz] T5 costmap           %.1f Hz\n", hz.t5_costmap);
            std::printf("[Hz] T6 astar             %.1f Hz\n", hz.t6_astar);
            last_hz_print = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    pipeline.stop();
    rclcpp::shutdown();
    return 0;
}
