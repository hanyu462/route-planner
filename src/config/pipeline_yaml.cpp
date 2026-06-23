#include "route_planner/config/pipeline_yaml.hpp"

#include <yaml-cpp/yaml.h>

namespace route_planner::config {

pipeline::PipelineConfig load_pipeline_config(const std::string& path)
{
    YAML::Node doc = YAML::LoadFile(path);
    YAML::Node n   = doc["pipeline"];

    pipeline::PipelineConfig cfg;
    cfg.t1_pointcloud2_spin_hz = n["t1_pointcloud2_spin_hz"].as<int>(100);
    cfg.t2_pose_spin_hz        = n["t2_pose_spin_hz"].as<int>(100);
    cfg.t3_processor_hz        = n["t3_processor_hz"].as<int>(100);
    cfg.t4_occupancy_grid_hz   = n["t4_occupancy_grid_hz"].as<int>(100);
    cfg.t5_costmap_hz          = n["t5_costmap_hz"].as<int>(100);
    cfg.t6_astar_hz            = n["t6_astar_hz"].as<int>(100);

    validate_pipeline_config(cfg);
    return cfg;
}

}  // namespace route_planner::config
