#include "route_planner/config/costmap_yaml.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace route_planner::config {

costmap::CostmapConfig load_costmap_config(const std::string& yaml_path)
{
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const YAML::Node c = root["costmap_builder"];

    if (!c) throw std::runtime_error("Missing 'costmap_builder' section in " + yaml_path);

    if (!c["lethal_radius"])    throw std::runtime_error("Missing 'lethal_radius' in "    + yaml_path);
    if (!c["influence_radius"]) throw std::runtime_error("Missing 'influence_radius' in " + yaml_path);
    if (!c["lethal_cost"])      throw std::runtime_error("Missing 'lethal_cost' in "      + yaml_path);
    if (!c["max_soft_cost"])    throw std::runtime_error("Missing 'max_soft_cost' in "    + yaml_path);
    if (!c["decay_factor"])     throw std::runtime_error("Missing 'decay_factor' in "     + yaml_path);

    costmap::CostmapConfig config;
    config.lethal_radius    = c["lethal_radius"].as<float>();
    config.influence_radius = c["influence_radius"].as<float>();
    config.lethal_cost      = static_cast<uint8_t>(c["lethal_cost"].as<int>());
    config.max_soft_cost    = static_cast<uint8_t>(c["max_soft_cost"].as<int>());
    config.decay_factor     = c["decay_factor"].as<float>();

    costmap::validate_costmap_config(config);
    return config;
}

}  // namespace route_planner::config
