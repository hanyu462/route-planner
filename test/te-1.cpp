// [TE-1] Raw PointCloudXYZFrame 확인 + RViz2 시각화
//
// 목적:
//   ROS2 PointCloud2 토픽을 수신해서 PointCloudXYZFrame으로 변환한 원시 데이터를 콘솔 출력
//   transform / filter 적용 전 데이터 확인용
//
// 실행:
//   ros2 run route_planner te_1_node
//   --ros-args --params-file config/pointcloud2_adapter.yaml
//
// RViz2:
//   ros2 run rviz2 rviz2
//   Fixed Frame: utlidar_lidar
//   Add → PointCloud2 → Topic: /route_planner/raw_points
//
// 출력:
//   stamp_ns, frame_id, point_count, 상위 5개 포인트 x/y/z

#include <cstdio>
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "route_planner/ros2/pointcloud2_adapter_node.hpp"
#include "route_planner/ros2/pointcloud2_adapter.hpp"

static void print_frame(const route_planner::common::PointCloudXYZFrame& frame)
{
    std::printf("\n── PointCloudXYZFrame ──────────────────\n");
    std::printf("  stamp_ns : %ld\n",  frame.stamp_ns);
    std::printf("  frame_id : %s\n",   frame.frame_id.c_str());
    std::printf("  points   : %zu\n",  frame.point_count());

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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto pub_node = rclcpp::Node::make_shared("pointcloud2_debug_node");
    auto pub = pub_node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/route_planner/raw_points", rclcpp::SensorDataQoS());

    auto buffer = std::make_shared<RawBuffer>();
    auto adapter = std::make_shared<PointCloud2AdapterNode>(buffer);

    std::thread spin_thread([&pub_node, &adapter]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(pub_node);
        exec.add_node(adapter);
        exec.spin();
    });

    uint64_t last_seq = 0;
    while (rclcpp::ok()) {
        if (auto snap = buffer->read_if_new(last_seq)) {
            last_seq = snap->sequence;
            pub->publish(route_planner::ros2::convert_xyz_frame_to_pointcloud2(*snap->value));
            print_frame(*snap->value);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
