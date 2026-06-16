#include "route_planner/config/pointcloud_processor_yaml.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace route_planner::config {

core::PointCloudProcessorConfig load_processor_config(const std::string& yaml_path)
{
    const YAML::Node root = YAML::LoadFile(yaml_path);

    const YAML::Node t = root["transform"];
    const YAML::Node f = root["filter"];

    if (!t) throw std::runtime_error("Missing 'transform' section in " + yaml_path);
    if (!f) throw std::runtime_error("Missing 'filter' section in " + yaml_path);

    core::PointCloudTransformOptions transform;
    transform.enabled  = t["enabled"].as<bool>();
    transform.offset_x = t["offset_x"].as<float>();
    transform.offset_y = t["offset_y"].as<float>();
    transform.offset_z = t["offset_z"].as<float>();
    transform.invert_x = t["invert_x"].as<bool>();
    transform.invert_y = t["invert_y"].as<bool>();
    transform.invert_z = t["invert_z"].as<bool>();

    core::PointCloudFilterOptions filter;
    filter.enabled = f["enabled"].as<bool>();
    filter.min_x   = f["min_x"].as<float>();
    filter.max_x   = f["max_x"].as<float>();
    filter.min_y   = f["min_y"].as<float>();
    filter.max_y   = f["max_y"].as<float>();
    filter.min_z   = f["min_z"].as<float>();
    filter.max_z   = f["max_z"].as<float>();

    core::PointCloudProcessorConfig config{transform, filter};
    core::validate_processor_config(config);

    return config;
}

}  // namespace route_planner::config
