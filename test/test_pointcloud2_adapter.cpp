#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "route_planner/ros2/pointcloud2_adapter.hpp"
#include "route_planner/core/latest_buffer.hpp"

using route_planner::ros2::convert_pointcloud2_to_xyz_frame;
using route_planner::ros2::PointCloud2AdapterOptions;
using route_planner::core::LatestBuffer;
using route_planner::core::PointCloudXYZFrame;

// ── helpers ──────────────────────────────────────────────────────────────────

static sensor_msgs::msg::PointField make_field(
    const std::string& name, uint32_t offset, uint8_t datatype)
{
    sensor_msgs::msg::PointField f;
    f.name     = name;
    f.offset   = offset;
    f.datatype = datatype;
    f.count    = 1;
    return f;
}

static sensor_msgs::msg::PointCloud2 make_xyz_cloud(
    const std::vector<std::array<float, 3>>& points)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "lidar";
    msg.header.stamp.sec    = 1;
    msg.header.stamp.nanosec = 500'000'000;

    msg.fields.push_back(make_field("x", 0,  sensor_msgs::msg::PointField::FLOAT32));
    msg.fields.push_back(make_field("y", 4,  sensor_msgs::msg::PointField::FLOAT32));
    msg.fields.push_back(make_field("z", 8,  sensor_msgs::msg::PointField::FLOAT32));

    msg.point_step = 12;
    msg.width      = static_cast<uint32_t>(points.size());
    msg.height     = 1;
    msg.row_step   = msg.point_step * msg.width;
    msg.is_dense   = true;

    msg.data.resize(msg.row_step);
    for (std::size_t i = 0; i < points.size(); ++i) {
        auto* ptr = msg.data.data() + i * msg.point_step;
        std::memcpy(ptr + 0, &points[i][0], sizeof(float));
        std::memcpy(ptr + 4, &points[i][1], sizeof(float));
        std::memcpy(ptr + 8, &points[i][2], sizeof(float));
    }

    return msg;
}

// ── convert_pointcloud2_to_xyz_frame ─────────────────────────────────────────

TEST(PointCloud2Adapter, ConvertsXYZCorrectly)
{
    auto msg = make_xyz_cloud({{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}});
    auto frame = convert_pointcloud2_to_xyz_frame(msg);

    ASSERT_EQ(frame.point_count(), 2u);
    EXPECT_FLOAT_EQ(frame.xyz[0], 1.0f);
    EXPECT_FLOAT_EQ(frame.xyz[1], 2.0f);
    EXPECT_FLOAT_EQ(frame.xyz[2], 3.0f);
    EXPECT_FLOAT_EQ(frame.xyz[3], 4.0f);
    EXPECT_FLOAT_EQ(frame.xyz[4], 5.0f);
    EXPECT_FLOAT_EQ(frame.xyz[5], 6.0f);
}

TEST(PointCloud2Adapter, PreservesStampAndFrameId)
{
    auto msg = make_xyz_cloud({{0.0f, 0.0f, 0.0f}});
    auto frame = convert_pointcloud2_to_xyz_frame(msg);

    EXPECT_EQ(frame.frame_id, "lidar");
    EXPECT_EQ(frame.stamp_ns, 1'500'000'000LL);
}

TEST(PointCloud2Adapter, MissingXYZFieldsReturnsEmpty)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.fields.push_back(make_field("intensity", 0, sensor_msgs::msg::PointField::FLOAT32));
    msg.point_step = 4;
    msg.width = 1; msg.height = 1;
    msg.data.resize(4);

    auto frame = convert_pointcloud2_to_xyz_frame(msg);
    EXPECT_TRUE(frame.empty());
}

TEST(PointCloud2Adapter, NonFloat32XYZFieldsReturnsEmpty)
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.fields.push_back(make_field("x", 0, sensor_msgs::msg::PointField::FLOAT64));
    msg.fields.push_back(make_field("y", 8, sensor_msgs::msg::PointField::FLOAT64));
    msg.fields.push_back(make_field("z", 16, sensor_msgs::msg::PointField::FLOAT64));
    msg.point_step = 24;
    msg.width = 1; msg.height = 1;
    msg.data.resize(24);

    auto frame = convert_pointcloud2_to_xyz_frame(msg);
    EXPECT_TRUE(frame.empty());
}

TEST(PointCloud2Adapter, RemoveInvalidPointsFiltersNaN)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto msg = make_xyz_cloud({{1.0f, 2.0f, 3.0f}, {nan, 0.0f, 0.0f}});

    PointCloud2AdapterOptions opts;
    opts.remove_invalid_points = true;
    auto frame = convert_pointcloud2_to_xyz_frame(msg, opts);

    ASSERT_EQ(frame.point_count(), 1u);
    EXPECT_FLOAT_EQ(frame.xyz[0], 1.0f);
}

TEST(PointCloud2Adapter, RemoveInvalidPointsFiltersInf)
{
    const float inf = std::numeric_limits<float>::infinity();
    auto msg = make_xyz_cloud({{inf, 0.0f, 0.0f}, {1.0f, 2.0f, 3.0f}});

    PointCloud2AdapterOptions opts;
    opts.remove_invalid_points = true;
    auto frame = convert_pointcloud2_to_xyz_frame(msg, opts);

    ASSERT_EQ(frame.point_count(), 1u);
    EXPECT_FLOAT_EQ(frame.xyz[0], 1.0f);
}

TEST(PointCloud2Adapter, KeepsInvalidPointsWhenDisabled)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto msg = make_xyz_cloud({{nan, nan, nan}, {1.0f, 2.0f, 3.0f}});

    PointCloud2AdapterOptions opts;
    opts.remove_invalid_points = false;
    auto frame = convert_pointcloud2_to_xyz_frame(msg, opts);

    EXPECT_EQ(frame.point_count(), 2u);
}

TEST(PointCloud2Adapter, EmptyCloudReturnsEmptyFrame)
{
    auto msg = make_xyz_cloud({});
    auto frame = convert_pointcloud2_to_xyz_frame(msg);
    EXPECT_TRUE(frame.empty());
}

// ── LatestBuffer ─────────────────────────────────────────────────────────────

TEST(LatestBuffer, ReadEmptyReturnsNullopt)
{
    LatestBuffer<PointCloudXYZFrame> buf;
    EXPECT_FALSE(buf.read().has_value());
}

TEST(LatestBuffer, WriteAndReadReturnsLatest)
{
    LatestBuffer<PointCloudXYZFrame> buf;

    PointCloudXYZFrame frame;
    frame.xyz = {1.0f, 2.0f, 3.0f};
    buf.write(frame);

    auto snap = buf.read();
    ASSERT_TRUE(snap.has_value());
    EXPECT_EQ(snap->value->xyz[0], 1.0f);
}

TEST(LatestBuffer, OverwriteKeepsOnlyLatest)
{
    LatestBuffer<PointCloudXYZFrame> buf;

    PointCloudXYZFrame a; a.xyz = {1.0f, 0.0f, 0.0f};
    PointCloudXYZFrame b; b.xyz = {9.0f, 0.0f, 0.0f};
    buf.write(a);
    buf.write(b);

    auto snap = buf.read();
    ASSERT_TRUE(snap.has_value());
    EXPECT_FLOAT_EQ(snap->value->xyz[0], 9.0f);
}

TEST(LatestBuffer, ReadIfNewReturnsNulloptWhenUnchanged)
{
    LatestBuffer<PointCloudXYZFrame> buf;
    PointCloudXYZFrame frame; frame.xyz = {1.0f, 0.0f, 0.0f};
    buf.write(frame);

    auto snap = buf.read();
    ASSERT_TRUE(snap.has_value());

    auto snap2 = buf.read_if_new(snap->sequence);
    EXPECT_FALSE(snap2.has_value());
}

TEST(LatestBuffer, ReadIfNewReturnsValueAfterWrite)
{
    LatestBuffer<PointCloudXYZFrame> buf;
    PointCloudXYZFrame a; a.xyz = {1.0f, 0.0f, 0.0f};
    buf.write(a);

    auto snap = buf.read();
    ASSERT_TRUE(snap.has_value());

    PointCloudXYZFrame b; b.xyz = {2.0f, 0.0f, 0.0f};
    buf.write(b);

    auto snap2 = buf.read_if_new(snap->sequence);
    ASSERT_TRUE(snap2.has_value());
    EXPECT_FLOAT_EQ(snap2->value->xyz[0], 2.0f);
}

TEST(LatestBuffer, SequenceIncrementsOnWrite)
{
    LatestBuffer<PointCloudXYZFrame> buf;
    PointCloudXYZFrame frame;

    buf.write(frame);
    auto s1 = buf.read()->sequence;
    buf.write(frame);
    auto s2 = buf.read()->sequence;

    EXPECT_EQ(s2, s1 + 1);
}
