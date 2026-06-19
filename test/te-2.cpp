// [TE-2] Processed PointCloudXYZFrame 확인 + RViz2 시각화
//
// 목적:
//   PointCloudRawBuffer에서 PointCloudXYZFrame을 읽어 PointCloudProcessor로 가공 후
//   PointCloudProcessedBuffer에 저장, ROS2 토픽 발행, 콘솔 출력
//
// 실행:
//   ros2 run route_planner te_2_node
//   --ros-args --params-file config/pointcloud2_adapter.yaml
//   -p processor_config:=config/pointcloud_processor.yaml
//
// RViz2:
//   ros2 run rviz2 rviz2
//   Fixed Frame: utlidar_lidar
//   Add → PointCloud2 → Topic: /route_planner/processed_points
//
// 출력:
//   raw point_count, processed point_count, 상위 5개 포인트 x/y/z

#include <cstdio>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/ros2/pointcloud2_adapter_node.hpp"
#include "route_planner/ros2/pointcloud2_adapter.hpp"
#include "route_planner/pointcloud/pointcloud_processor.hpp"
#include "route_planner/config/pointcloud_processor_yaml.hpp"

using route_planner::pointcloud::PointCloudProcessedBuffer;

static void print_frame(const route_planner::common::PointCloudXYZFrame& raw,
                        const route_planner::common::PointCloudXYZFrame& processed)
{
    std::printf("\n── PointCloudXYZFrame (processed) ──────\n");
    std::printf("  stamp_ns  : %ld\n", processed.stamp_ns);
    std::printf("  frame_id  : %s\n",  processed.frame_id.c_str());
    std::printf("  raw pts   : %zu\n", raw.point_count());
    std::printf("  after flt : %zu\n", processed.point_count());

    constexpr std::size_t PREVIEW = 5;
    const std::size_t n = std::min(processed.point_count(), PREVIEW);
    for (std::size_t i = 0; i < n; ++i) {
        std::printf("  [%zu] x=%.4f  y=%.4f  z=%.4f\n",
            i,
            processed.xyz[i * 3 + 0],
            processed.xyz[i * 3 + 1],
            processed.xyz[i * 3 + 2]);
    }
    if (processed.point_count() > PREVIEW) {
        std::printf("  ... (%zu more)\n", processed.point_count() - PREVIEW);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("pointcloud2_processor_node");
    const std::string config_path =
        node->declare_parameter<std::string>("processor_config",
                                             "config/pointcloud_processor.yaml");

    route_planner::pointcloud::PointCloudProcessor processor{
        route_planner::config::load_processor_config(config_path)};

    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/route_planner/processed_points", rclcpp::SensorDataQoS());

    auto raw_buffer       = std::make_shared<PointCloudRawBuffer>();
    auto processed_buffer = std::make_shared<PointCloudProcessedBuffer>();
    auto adapter = std::make_shared<PointCloud2AdapterNode>(raw_buffer);

    std::thread spin_thread([&adapter]() { rclcpp::spin(adapter); });

    uint64_t last_seq = 0;
    while (rclcpp::ok()) {
        if (auto snap = raw_buffer->read_if_new(last_seq)) {
            last_seq = snap->sequence;
            const auto processed = processor.process(*snap->value);
            processed_buffer->write(processed);
            pub->publish(route_planner::ros2::convert_xyz_frame_to_pointcloud2(processed));
            print_frame(*snap->value, processed);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
