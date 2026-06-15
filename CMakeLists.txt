cmake_minimum_required(VERSION 3.16)
project(route_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── adapter library ──────────────────────────────────────────────────────────
add_library(pointcloud2_adapter
  src/ros2/pointcloud2_adapter.cpp
)
target_include_directories(pointcloud2_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pointcloud2_adapter
  sensor_msgs
  builtin_interfaces
)

# ── adapter node ─────────────────────────────────────────────────────────────
add_executable(pointcloud2_adapter_node
  src/ros2/pointcloud2_adapter_node.cpp
)
target_link_libraries(pointcloud2_adapter_node
  pointcloud2_adapter
)
ament_target_dependencies(pointcloud2_adapter_node
  rclcpp
  sensor_msgs
)

# ── debug node ───────────────────────────────────────────────────────────────
add_executable(pointcloud2_debug_node
  test/pointcloud2_debug_node.cpp
)
target_link_libraries(pointcloud2_debug_node
  pointcloud2_adapter
)
ament_target_dependencies(pointcloud2_debug_node
  rclcpp
  sensor_msgs
)

# ── install ──────────────────────────────────────────────────────────────────
install(TARGETS pointcloud2_adapter pointcloud2_adapter_node pointcloud2_debug_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
