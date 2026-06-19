#include "route_planner/config/occupancy_grid_yaml.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace route_planner::config {

occupancy_grid::OccupancyGridConfig load_occupancy_grid_config(const std::string& yaml_path)
{
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const YAML::Node g = root["occupancy_grid"];

    if (!g) throw std::runtime_error("Missing 'occupancy_grid' section in " + yaml_path);

    if (!g["resolution"])
        throw std::runtime_error("Missing 'resolution' in " + yaml_path);
    if (!g["align_to_pose_frame"])
        throw std::runtime_error("Missing 'align_to_pose_frame' in " + yaml_path);
    if (!g["cell_values"])
        throw std::runtime_error("Missing 'cell_values' section in " + yaml_path);
    if (!g["min_x"] || !g["max_x"])
        throw std::runtime_error("Missing 'min_x' or 'max_x' in " + yaml_path);
    if (!g["min_y"] || !g["max_y"])
        throw std::runtime_error("Missing 'min_y' or 'max_y' in " + yaml_path);

    const YAML::Node cv = g["cell_values"];
    if (!cv["unknown"] || !cv["free"] || !cv["occupied"])
        throw std::runtime_error("Missing field(s) in 'cell_values' section in " + yaml_path);

    occupancy_grid::OccupancyGridConfig config;
    config.resolution      = g["resolution"].as<float>();
    config.align_to_pose_frame = g["align_to_pose_frame"].as<bool>();
    config.cell_values     = {
        static_cast<int8_t>(cv["unknown"].as<int>()),
        static_cast<int8_t>(cv["free"].as<int>()),
        static_cast<int8_t>(cv["occupied"].as<int>()),
    };
    config.min_x           = g["min_x"].as<float>();
    config.max_x           = g["max_x"].as<float>();
    config.min_y           = g["min_y"].as<float>();
    config.max_y           = g["max_y"].as<float>();

    occupancy_grid::validate_occupancy_grid_config(config);
    return config;
}

}  // namespace route_planner::config
