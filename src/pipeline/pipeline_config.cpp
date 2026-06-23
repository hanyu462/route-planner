#include "route_planner/pipeline/route_planner_pipeline_config.hpp"

#include <stdexcept>

namespace route_planner::pipeline {

void validate_pipeline_config(const PipelineConfig& cfg)
{
    if (cfg.t1_pointcloud2_spin_hz <= 0) throw std::invalid_argument("t1_pointcloud2_spin_hz must be > 0");
    if (cfg.t2_pose_spin_hz        <= 0) throw std::invalid_argument("t2_pose_spin_hz must be > 0");
    if (cfg.t3_processor_hz        <= 0) throw std::invalid_argument("t3_processor_hz must be > 0");
    if (cfg.t4_occupancy_grid_hz   <= 0) throw std::invalid_argument("t4_occupancy_grid_hz must be > 0");
    if (cfg.t5_costmap_hz          <= 0) throw std::invalid_argument("t5_costmap_hz must be > 0");
    if (cfg.t6_astar_hz            <= 0) throw std::invalid_argument("t6_astar_hz must be > 0");
}

}  // namespace route_planner::pipeline
