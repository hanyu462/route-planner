#pragma once

#include <string>
#include "route_planner/pipeline/route_planner_pipeline_config.hpp"

namespace route_planner::config {

pipeline::PipelineConfig load_pipeline_config(const std::string& path);

}  // namespace route_planner::config
