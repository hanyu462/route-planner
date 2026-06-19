#pragma once

#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>

namespace route_planner::ros2 {

inline int64_t to_nanoseconds(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL +
           static_cast<int64_t>(stamp.nanosec);
}

}  // namespace route_planner::ros2
