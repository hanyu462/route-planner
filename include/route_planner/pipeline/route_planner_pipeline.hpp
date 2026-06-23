#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "route_planner/common/hz_counter.hpp"
#include "route_planner/common/latest_buffer.hpp"
#include "route_planner/common/pointcloud_xyz_frame.hpp"
#include "route_planner/common/pose_xy.hpp"
#include "route_planner/pointcloud/pointcloud_processor.hpp"
#include "route_planner/occupancy_grid/occupancy_grid_builder.hpp"
#include "route_planner/costmap/costmap_builder.hpp"
#include "route_planner/astar/astar_planner.hpp"
#include "route_planner/common/path.hpp"
#include "route_planner/ros2/pointcloud2_adapter_node.hpp"
#include "route_planner/ros2/pose_adapter_node.hpp"

namespace route_planner::pipeline {

class RoutePlannerPipeline {
public:
    struct Config {
        std::string processor_config_path;
        std::string grid_config_path;
        std::string costmap_config_path;
        std::string astar_config_path;
        std::string pipeline_config_path;
    };

    explicit RoutePlannerPipeline(const Config& config);
    ~RoutePlannerPipeline();

    RoutePlannerPipeline(const RoutePlannerPipeline&) = delete;
    RoutePlannerPipeline& operator=(const RoutePlannerPipeline&) = delete;

    void start();
    void stop();

    void set_goal(float map_x, float map_y);

    struct ThreadHz {
        float t1_pointcloud2_spin;
        float t2_pose_spin;
        float t3_processor;
        float t4_occupancy_grid;
        float t5_costmap;
        float t6_astar;
    };
    ThreadHz thread_hz() const;

    // Buffer accessors for visualization (te-5-2 test node)
    std::shared_ptr<PointCloudRawBuffer>                         pointcloud_raw_buffer()       const { return pointcloud_raw_buffer_; }
    std::shared_ptr<PoseBuffer>                                  pose_buffer()                 const { return pose_buffer_; }
    std::shared_ptr<pointcloud::PointCloudProcessedBuffer>       pointcloud_processed_buffer() const { return pointcloud_processed_buffer_; }
    std::shared_ptr<occupancy_grid::OccupancyGridBuffer>         occupancy_grid_buffer()       const { return occupancy_grid_buffer_; }
    std::shared_ptr<costmap::CostmapBuffer>                      costmap_buffer()              const { return costmap_buffer_; }
    std::shared_ptr<common::PathBuffer>                          path_buffer()                 const { return path_buffer_; }

private:

    // Pipeline components
    pointcloud::PointCloudProcessor      pointcloud_processor_;
    occupancy_grid::OccupancyGridBuilder occupancy_grid_builder_;
    costmap::CostmapBuilder              costmap_builder_;
    astar::AStarPlanner                  astar_planner_;

    // Buffers
    std::shared_ptr<PointCloudRawBuffer>                    pointcloud_raw_buffer_;
    std::shared_ptr<PoseBuffer>                             pose_buffer_;
    std::shared_ptr<pointcloud::PointCloudProcessedBuffer>  pointcloud_processed_buffer_;
    std::shared_ptr<occupancy_grid::OccupancyGridBuffer>    occupancy_grid_buffer_;
    std::shared_ptr<costmap::CostmapBuffer>                 costmap_buffer_;
    std::shared_ptr<common::PathBuffer>                     path_buffer_;

    // Goal (thread-safe)
    mutable std::mutex                    goal_mutex_;
    std::optional<std::pair<float,float>> goal_;

    // ROS2 adapter nodes
    std::shared_ptr<PointCloud2AdapterNode> pc2_adapter_node_;
    std::shared_ptr<PoseAdapterNode>        pose_adapter_node_;

    // Thread Hz counters
    common::HzCounter t1_hz_;
    common::HzCounter t2_hz_;
    common::HzCounter t3_hz_;
    common::HzCounter t4_hz_;
    common::HzCounter t5_hz_;
    common::HzCounter t6_hz_;

    // Thread sleep durations (computed from config Hz)
    std::chrono::milliseconds t1_sleep_;
    std::chrono::milliseconds t2_sleep_;
    std::chrono::milliseconds t3_sleep_;
    std::chrono::milliseconds t4_sleep_;
    std::chrono::milliseconds t5_sleep_;
    std::chrono::milliseconds t6_sleep_;

    // Threads
    std::atomic<bool> running_{false};
    std::thread t1_pointcloud2_spin_;
    std::thread t2_pose_spin_;
    std::thread t3_processor_;
    std::thread t4_occupancy_grid_;
    std::thread t5_costmap_;
    std::thread t6_astar_;

    void run_t3();
    void run_t4();
    void run_t5();
    void run_t6();
};

}  // namespace route_planner::pipeline
