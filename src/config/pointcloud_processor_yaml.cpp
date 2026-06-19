#include "route_planner/config/pointcloud_processor_yaml.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace route_planner::config {

pointcloud::PointCloudProcessorConfig load_processor_config(const std::string& yaml_path)
{
    const YAML::Node root = YAML::LoadFile(yaml_path);

    const YAML::Node t = root["transform"];
    const YAML::Node f = root["filter"];

    if (!t) throw std::runtime_error("Missing 'transform' section in " + yaml_path);
    if (!f) throw std::runtime_error("Missing 'filter' section in " + yaml_path);

    pointcloud::PointCloudTransformOptions transform;
    transform.enabled  = t["enabled"].as<bool>();
    transform.offset_x = t["offset_x"].as<float>();
    transform.offset_y = t["offset_y"].as<float>();
    transform.offset_z = t["offset_z"].as<float>();
    transform.invert_x = t["invert_x"].as<bool>();
    transform.invert_y = t["invert_y"].as<bool>();
    transform.invert_z = t["invert_z"].as<bool>();

    pointcloud::PointCloudFilterOptions filter;
    filter.enabled       = f["enabled"].as<bool>();

    filter.enable_x      = f["enable_x"].as<bool>();
    filter.min_x         = f["min_x"].as<float>();
    filter.max_x         = f["max_x"].as<float>();

    filter.enable_y      = f["enable_y"].as<bool>();
    filter.min_y         = f["min_y"].as<float>();
    filter.max_y         = f["max_y"].as<float>();

    filter.enable_z      = f["enable_z"].as<bool>();
    filter.min_z         = f["min_z"].as<float>();
    filter.max_z         = f["max_z"].as<float>();

    filter.enable_radius = f["enable_radius"].as<bool>();
    filter.min_radius    = f["min_radius"].as<float>();

    pointcloud::PointCloudProcessorConfig config{transform, filter};
    pointcloud::validate_processor_config(config);

    return config;
}

}  // namespace route_planner::config
