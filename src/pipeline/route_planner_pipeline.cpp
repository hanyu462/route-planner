#include "route_planner/pipeline/route_planner_pipeline.hpp"

#include <chrono>
#include <cstdio>
#include <thread>


#include "route_planner/config/pointcloud_processor_yaml.hpp"
#include "route_planner/config/occupancy_grid_yaml.hpp"
#include "route_planner/config/costmap_yaml.hpp"
#include "route_planner/config/astar_yaml.hpp"
#include "route_planner/config/pipeline_yaml.hpp"

namespace route_planner::pipeline {

// ── helpers ───────────────────────────────────────────────────────────────────

static occupancy_grid::OccupancyGridConfig
load_grid_config(const std::string& path)
{
    return config::load_occupancy_grid_config(path);
}

static std::chrono::milliseconds hz_to_ms(int hz)
{
    return std::chrono::milliseconds(1000 / hz);
}

// ── constructor / destructor ──────────────────────────────────────────────────

RoutePlannerPipeline::RoutePlannerPipeline(const Config& cfg)
    : pointcloud_processor_    (config::load_processor_config(cfg.processor_config_path))
    , occupancy_grid_builder_  (load_grid_config(cfg.grid_config_path))
    , costmap_builder_         (config::load_costmap_config(cfg.costmap_config_path))
    , astar_planner_           (config::load_astar_config(cfg.astar_config_path))
{
    const auto pipeline_cfg = config::load_pipeline_config(cfg.pipeline_config_path);
    t1_sleep_ = hz_to_ms(pipeline_cfg.t1_pointcloud2_spin_hz);
    t2_sleep_ = hz_to_ms(pipeline_cfg.t2_pose_spin_hz);
    t3_sleep_ = hz_to_ms(pipeline_cfg.t3_processor_hz);
    t4_sleep_ = hz_to_ms(pipeline_cfg.t4_occupancy_grid_hz);
    t5_sleep_ = hz_to_ms(pipeline_cfg.t5_costmap_hz);
    t6_sleep_ = hz_to_ms(pipeline_cfg.t6_astar_hz);
    pointcloud_raw_buffer_          = std::make_shared<PointCloudRawBuffer>();
    pose_buffer_                    = std::make_shared<PoseBuffer>();
    pointcloud_processed_buffer_    = std::make_shared<pointcloud::PointCloudProcessedBuffer>();
    occupancy_grid_buffer_          = std::make_shared<occupancy_grid::OccupancyGridBuffer>();
    costmap_buffer_                 = std::make_shared<costmap::CostmapBuffer>();
    path_buffer_                    = std::make_shared<common::PathBuffer>();

    pc2_adapter_node_  = std::make_shared<PointCloud2AdapterNode>(pointcloud_raw_buffer_);
    pose_adapter_node_ = std::make_shared<PoseAdapterNode>(pose_buffer_);
}

RoutePlannerPipeline::~RoutePlannerPipeline()
{
    stop();
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

void RoutePlannerPipeline::start()
{
    if (running_.exchange(true)) return;

    t1_pointcloud2_spin_ = std::thread([this] {
        while (running_) {
            const auto start = std::chrono::steady_clock::now();
            rclcpp::spin_some(pc2_adapter_node_);
            t1_hz_.tick();
            const auto remaining = t1_sleep_ - (std::chrono::steady_clock::now() - start);
            if (remaining > std::chrono::nanoseconds::zero()) {
                std::this_thread::sleep_for(remaining);
            }
        }
    });

    t2_pose_spin_ = std::thread([this] {
        while (running_) {
            const auto start = std::chrono::steady_clock::now();
            rclcpp::spin_some(pose_adapter_node_);
            t2_hz_.tick();
            const auto remaining = t2_sleep_ - (std::chrono::steady_clock::now() - start);
            if (remaining > std::chrono::nanoseconds::zero()) {
                std::this_thread::sleep_for(remaining);
            }
        }
    });

    t3_processor_ = std::thread([this] { run_t3(); });
    t4_occupancy_grid_      = std::thread([this] { run_t4(); });
    t5_costmap_   = std::thread([this] { run_t5(); });
    t6_astar_     = std::thread([this] { run_t6(); });
}

void RoutePlannerPipeline::stop()
{
    running_.store(false);

    if (t1_pointcloud2_spin_.joinable())  t1_pointcloud2_spin_.join();
    if (t2_pose_spin_.joinable()) t2_pose_spin_.join();
    if (t3_processor_.joinable()) t3_processor_.join();
    if (t4_occupancy_grid_.joinable())      t4_occupancy_grid_.join();
    if (t5_costmap_.joinable())   t5_costmap_.join();
    if (t6_astar_.joinable())     t6_astar_.join();
}

// ── goal ──────────────────────────────────────────────────────────────────────

RoutePlannerPipeline::ThreadHz RoutePlannerPipeline::thread_hz() const
{
    return {
        t1_hz_.hz(), t2_hz_.hz(),
        t3_hz_.hz(), t4_hz_.hz(),
        t5_hz_.hz(), t6_hz_.hz()
    };
}

void RoutePlannerPipeline::set_goal(float map_x, float map_y)
{
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_ = {map_x, map_y};
}

// ── T3: PointCloud Processor ──────────────────────────────────────────────────

void RoutePlannerPipeline::run_t3()
{
    while (running_) {
        const auto start = std::chrono::steady_clock::now();
        if (auto snap = pointcloud_raw_buffer_->read()) {
            pointcloud_processed_buffer_->write(pointcloud_processor_.process(*snap->value));
            t3_hz_.tick();
        }
        const auto remaining = t3_sleep_ - (std::chrono::steady_clock::now() - start);
        if (remaining > std::chrono::nanoseconds::zero()) {
            std::this_thread::sleep_for(remaining);
        }
    }
}

// ── T4: OccupancyGrid Builder ─────────────────────────────────────────────────

void RoutePlannerPipeline::run_t4()
{
    while (running_) {
        const auto start = std::chrono::steady_clock::now();
        if (auto snap = pointcloud_processed_buffer_->read()) {
            occupancy_grid_buffer_->write(occupancy_grid_builder_.build(*snap->value));
            t4_hz_.tick();
        }
        const auto remaining = t4_sleep_ - (std::chrono::steady_clock::now() - start);
        if (remaining > std::chrono::nanoseconds::zero()) {
            std::this_thread::sleep_for(remaining);
        }
    }
}

// ── T5: CostmapBuilder ────────────────────────────────────────────────────────

void RoutePlannerPipeline::run_t5()
{
    while (running_) {
        const auto start = std::chrono::steady_clock::now();
        if (auto snap = occupancy_grid_buffer_->read()) {
            costmap_buffer_->write(costmap_builder_.build(*snap->value));
            t5_hz_.tick();
        }
        const auto remaining = t5_sleep_ - (std::chrono::steady_clock::now() - start);
        if (remaining > std::chrono::nanoseconds::zero()) {
            std::this_thread::sleep_for(remaining);
        }
    }
}

// ── T6: A* Planner ────────────────────────────────────────────────────────────

void RoutePlannerPipeline::run_t6()
{
    while (running_) {
        const auto start = std::chrono::steady_clock::now();

        if (auto costmap_snap = costmap_buffer_->read()) {
            t6_hz_.tick();

            std::optional<std::pair<float,float>> goal;
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                goal = goal_;
            }

            std::optional<common::PoseXY> pose;
            if (auto snap = pose_buffer_->read()) {
                pose = *snap->value;
            }

            if (goal && pose) {
                path_buffer_->write(
                    astar_planner_.plan(*costmap_snap->value, *goal, pose));
            }
        }

        const auto remaining = t6_sleep_ - (std::chrono::steady_clock::now() - start);
        if (remaining > std::chrono::nanoseconds::zero()) {
            std::this_thread::sleep_for(remaining);
        }
    }
}

}  // namespace route_planner::pipeline
