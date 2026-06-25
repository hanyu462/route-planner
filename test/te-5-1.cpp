// [TE-5-1] Costmap 생성 및 RViz2 시각화
//
// 목적:
//   te-4 파이프라인에 CostmapBuilder를 추가하여 OccupancyGrid로부터
//   Euclidean Distance Transform 기반 비용 맵을 생성하고 시각화한다.
//   장애물 주변에 lethal zone과 soft zone이 형성되는지 확인한다.
//
// 실행:
//   ros2 run route_planner te_5_1_node \
//     --ros-args \
//     --params-file config/pointcloud2_adapter.yaml \
//     --params-file config/pose_adapter.yaml \
//     -p processor_config:=config/pointcloud_processor.yaml \
//     -p grid_config:=config/occupancy_grid_builder.yaml \
//     -p costmap_config:=config/costmap_builder.yaml
//
// RViz2:
//   Fixed Frame: map
//   Add → Map  → Topic: /route_planner/occupancy_grid  (이진 장애물 맵)
//   Add → Map  → Topic: /route_planner/costmap         (비용 분포, 0~100)
//   Add → Pose → Topic: /route_planner/pose
//
// 출력:
//   costmap 통계: lethal cells (cost >= lethal_cost), soft zone cells (0 < cost < lethal_cost)

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
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/common/pose_xy.hpp"
#include "route_planner/ros2/pointcloud2_adapter_node.hpp"
#include "route_planner/ros2/pose_adapter_node.hpp"
#include "route_planner/ros2/pose_adapter.hpp"
#include "route_planner/pointcloud/pointcloud_processor.hpp"
#include "route_planner/occupancy_grid/occupancy_grid_builder.hpp"
#include "route_planner/costmap/costmap_builder.hpp"
#include "route_planner/config/pointcloud_processor_yaml.hpp"
#include "route_planner/config/occupancy_grid_yaml.hpp"
#include "route_planner/config/costmap_yaml.hpp"

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

    // Normalize uint8 cell [0, lethal_cost] → int8 [0, 100] for RViz2
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

static void print_costmap(const route_planner::costmap::Costmap& costmap,
                           uint8_t lethal_cost)
{
    int lethal_cells = 0;
    int soft_cells   = 0;
    for (uint8_t c : costmap.cells) {
        if (c >= lethal_cost) ++lethal_cells;
        else if (c > 0)       ++soft_cells;
    }

    std::printf("\n── Costmap ──────────────────────────────\n");
    std::printf("  size     : %d x %d\n", costmap.width, costmap.height);
    std::printf("  lethal   : %d cells  (cost >= %d)\n", lethal_cells, lethal_cost);
    std::printf("  soft zone: %d cells  (0 < cost < %d)\n", soft_cells, lethal_cost);
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("costmap_node");

    const std::string processor_config_path =
        node->declare_parameter<std::string>("processor_config",
                                             "config/pointcloud_processor.yaml");
    const std::string grid_config_path =
        node->declare_parameter<std::string>("grid_config",
                                             "config/occupancy_grid_builder.yaml");
    const std::string costmap_config_path =
        node->declare_parameter<std::string>("costmap_config",
                                             "config/costmap_builder.yaml");

    const auto processor_config =
        route_planner::config::load_processor_config(processor_config_path);
    const auto grid_config =
        route_planner::config::load_occupancy_grid_config(grid_config_path);
    const auto costmap_config =
        route_planner::config::load_costmap_config(costmap_config_path);

    route_planner::pointcloud::PointCloudProcessor processor{processor_config};
    route_planner::occupancy_grid::OccupancyGridBuilder grid_builder{grid_config};
    route_planner::costmap::CostmapBuilder costmap_builder{costmap_config};

    auto grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/route_planner/occupancy_grid", rclcpp::SensorDataQoS());
    auto costmap_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/route_planner/costmap", rclcpp::SensorDataQoS());
    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/route_planner/pose", rclcpp::SensorDataQoS());

    auto raw_buffer       = std::make_shared<PointCloudRawBuffer>();
    auto processed_buffer = std::make_shared<PointCloudProcessedBuffer>();
    auto grid_buffer      = std::make_shared<OccupancyGridBuffer>();

    auto adapter = std::make_shared<PointCloud2AdapterNode>(raw_buffer);
    std::thread spin_thread([&adapter]() { rclcpp::spin(adapter); });

    std::shared_ptr<PoseBuffer>        pos_buffer;
    std::shared_ptr<PoseAdapterNode>   uwb_node;
    std::thread                        uwb_spin_thread;

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

                print_costmap(costmap, costmap_config.lethal_cost);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spin_thread.join();
    if (uwb_spin_thread.joinable()) uwb_spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
