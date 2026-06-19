#include "route_planner/ros2/pointcloud2_adapter.hpp"
#include "route_planner/ros2/stamp_utils.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstring>
#include <optional>

namespace {

struct XYZOffsets {
    uint32_t x;
    uint32_t y;
    uint32_t z;
};

std::optional<XYZOffsets> find_xyz_offsets(
    const std::vector<sensor_msgs::msg::PointField>& fields)
{
    XYZOffsets offsets{};
    bool found_x = false, found_y = false, found_z = false;

    for (const auto& f : fields) {
        if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {
            continue;
        }
        if (f.name == "x") { offsets.x = f.offset; found_x = true; }
        else if (f.name == "y") { offsets.y = f.offset; found_y = true; }
        else if (f.name == "z") { offsets.z = f.offset; found_z = true; }
    }

    if (!found_x || !found_y || !found_z) {
        return std::nullopt;
    }
    return offsets;
}

}  // namespace

namespace route_planner::ros2 {

route_planner::common::PointCloudXYZFrame convert_pointcloud2_to_xyz_frame(
    const sensor_msgs::msg::PointCloud2& msg,
    const PointCloud2AdapterOptions& options)
{
    const auto offsets = find_xyz_offsets(msg.fields);
    if (!offsets) {
        return {};
    }

    route_planner::common::PointCloudXYZFrame frame;
    frame.stamp_ns = to_nanoseconds(msg.header.stamp);
    frame.frame_id = msg.header.frame_id;

    const std::size_t n_points = msg.width * msg.height;
    frame.xyz.reserve(n_points * 3);

    for (std::size_t i = 0; i < n_points; ++i) {
        const auto* ptr = msg.data.data() + i * msg.point_step;

        float x, y, z;
        std::memcpy(&x, ptr + offsets->x, sizeof(float));
        std::memcpy(&y, ptr + offsets->y, sizeof(float));
        std::memcpy(&z, ptr + offsets->z, sizeof(float));

        if (options.remove_invalid_points && !std::isfinite(x + y + z)) {
            continue;
        }
        frame.xyz.insert(frame.xyz.end(), {x, y, z});
    }

    return frame;
}

sensor_msgs::msg::PointCloud2 convert_xyz_frame_to_pointcloud2(
    const route_planner::common::PointCloudXYZFrame& frame)
{
    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp.sec     = static_cast<int32_t>(frame.stamp_ns / 1'000'000'000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(frame.stamp_ns % 1'000'000'000LL);
    msg.header.frame_id      = frame.frame_id;

    msg.height     = 1;
    msg.width      = static_cast<uint32_t>(frame.point_count());
    msg.point_step = 12;
    msg.row_step   = msg.point_step * msg.width;
    msg.is_bigendian = false;
    msg.is_dense     = true;

    for (const auto& [name, offset] : std::initializer_list<std::pair<const char*, uint32_t>>{
            {"x", 0}, {"y", 4}, {"z", 8}}) {
        sensor_msgs::msg::PointField f;
        f.name     = name;
        f.offset   = offset;
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count    = 1;
        msg.fields.push_back(f);
    }

    const std::size_t n_bytes = frame.xyz.size() * sizeof(float);
    msg.data.resize(n_bytes);
    std::memcpy(msg.data.data(), frame.xyz.data(), n_bytes);

    return msg;
}

}  // namespace route_planner::ros2
