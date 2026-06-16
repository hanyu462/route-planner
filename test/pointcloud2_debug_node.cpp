#include <cstdio>
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "route_planner/ros2/pointcloud2_adapter_node.hpp"

static void print_frame(const route_planner::core::PointCloudXYZFrame& frame)
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

    auto buffer = std::make_shared<FrameBuffer>();
    auto node   = std::make_shared<PointCloud2AdapterNode>(buffer);

    std::thread spin_thread([&node]() { rclcpp::spin(node); });

    uint64_t last_seq = 0;
    while (rclcpp::ok()) {
        if (auto snap = buffer->read_if_new(last_seq)) {
            last_seq = snap->sequence;
            print_frame(*snap->value);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
