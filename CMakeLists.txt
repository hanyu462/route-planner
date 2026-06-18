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
find_package(yaml-cpp REQUIRED)

# ── core library ─────────────────────────────────────────────────────────────
add_library(route_planner_core
  src/pointcloud/pointcloud_transform.cpp
  src/pointcloud/pointcloud_filter.cpp
  src/pointcloud/pointcloud_processor.cpp
  src/pointcloud/pointcloud_processor_config.cpp
  src/config/pointcloud_processor_yaml.cpp
)
target_include_directories(route_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(route_planner_core
  yaml-cpp
)

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

# ── te-1 node (raw frames) ───────────────────────────────────────────────────
add_executable(te_1_node
  test/te-1.cpp
)
target_link_libraries(te_1_node
  pointcloud2_adapter
)
ament_target_dependencies(te_1_node
  rclcpp
  sensor_msgs
)

# ── te-2 node (processed frames) ─────────────────────────────────────────────
add_executable(te_2_node
  test/te-2.cpp
)
target_link_libraries(te_2_node
  pointcloud2_adapter
  route_planner_core
)
ament_target_dependencies(te_2_node
  rclcpp
  sensor_msgs
)

# ── install ──────────────────────────────────────────────────────────────────
install(TARGETS route_planner_core pointcloud2_adapter pointcloud2_adapter_node te_1_node te_2_node
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
