#include "route_planner/config/astar_yaml.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace route_planner::config {

astar::AStarConfig load_astar_config(const std::string& yaml_path)
{
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const YAML::Node a    = root["astar"];

    if (!a) throw std::runtime_error("Missing 'astar' section in " + yaml_path);

    if (!a["base_cost"])      throw std::runtime_error("Missing 'astar.base_cost' in "      + yaml_path);
    if (!a["obs_cost"])       throw std::runtime_error("Missing 'astar.obs_cost' in "       + yaml_path);
    if (!a["astar_weight"])   throw std::runtime_error("Missing 'astar.astar_weight' in "   + yaml_path);
    if (!a["max_iterations"]) throw std::runtime_error("Missing 'astar.max_iterations' in " + yaml_path);

    astar::AStarConfig config;
    config.base_cost      = a["base_cost"].as<float>();
    config.obs_cost       = a["obs_cost"].as<float>();
    config.astar_weight   = a["astar_weight"].as<float>();
    config.max_iterations = a["max_iterations"].as<int>();

    astar::validate_astar_config(config);
    return config;
}

}  // namespace route_planner::config
