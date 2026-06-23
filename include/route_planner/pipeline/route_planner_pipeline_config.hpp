#pragma once

namespace route_planner::pipeline {

struct PipelineConfig {
    int t1_pointcloud2_spin_hz = 100;
    int t2_pose_spin_hz        = 100;
    int t3_processor_hz        = 100;
    int t4_occupancy_grid_hz   = 100;
    int t5_costmap_hz          = 100;
    int t6_astar_hz            = 100;
};

void validate_pipeline_config(const PipelineConfig& config);

}  // namespace route_planner::pipeline
