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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Required for linking static libs into pybind11 shared object
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── core library ─────────────────────────────────────────────────────────────
add_library(route_planner_core
  src/pointcloud/pointcloud_transform.cpp
  src/pointcloud/pointcloud_filter.cpp
  src/pointcloud/pointcloud_processor.cpp
  src/pointcloud/pointcloud_processor_config.cpp
  src/config/pointcloud_processor_yaml.cpp
  src/occupancy_grid/occupancy_grid_config.cpp
  src/occupancy_grid/occupancy_grid_builder.cpp
  src/config/occupancy_grid_yaml.cpp
  src/costmap/costmap_config.cpp
  src/costmap/euclidean_distance_transform.cpp
  src/costmap/costmap_builder.cpp
  src/config/costmap_yaml.cpp
  src/astar/astar_config.cpp
  src/astar/astar_planner.cpp
  src/config/astar_yaml.cpp
  src/pipeline/pipeline_config.cpp
  src/config/pipeline_yaml.cpp
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

# ── pose adapter library ──────────────────────────────────────────────────────
add_library(pose_adapter
  src/ros2/pose_adapter.cpp
)
target_include_directories(pose_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pose_adapter
  geometry_msgs
)

# ── pose adapter node ─────────────────────────────────────────────────────────
add_executable(pose_adapter_node
  src/ros2/pose_adapter_node.cpp
)
target_link_libraries(pose_adapter_node
  pose_adapter
)
ament_target_dependencies(pose_adapter_node
  rclcpp
  geometry_msgs
)

# ── te-3 node (occupancy grid) ───────────────────────────────────────────────
add_executable(te_3_node
  test/te-3.cpp
)
target_link_libraries(te_3_node
  pointcloud2_adapter
  route_planner_core
)
ament_target_dependencies(te_3_node
  rclcpp
  sensor_msgs
  nav_msgs
)

# ── te-4 node (occupancy grid + UWB position offset) ─────────────────────────
add_executable(te_4_node
  test/te-4.cpp
)
target_link_libraries(te_4_node
  pointcloud2_adapter
  pose_adapter
  route_planner_core
)
ament_target_dependencies(te_4_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# ── te-5-1 node (costmap) ────────────────────────────────────────────────────
add_executable(te_5_1_node
  test/te-5-1.cpp
)
target_link_libraries(te_5_1_node
  pointcloud2_adapter
  pose_adapter
  route_planner_core
)
ament_target_dependencies(te_5_1_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# ── te-5-2 node (A* path planner) ────────────────────────────────────────────
add_executable(te_5_2_node
  test/te-5-2.cpp
)
target_link_libraries(te_5_2_node
  pointcloud2_adapter
  pose_adapter
  route_planner_core
)
ament_target_dependencies(te_5_2_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# ── pipeline library ─────────────────────────────────────────────────────────
add_library(route_planner_pipeline
  src/pipeline/route_planner_pipeline.cpp
)
target_include_directories(route_planner_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(route_planner_pipeline
  route_planner_core
  pointcloud2_adapter
  pose_adapter
)
ament_target_dependencies(route_planner_pipeline
  rclcpp
  sensor_msgs
  geometry_msgs
)

# ── te-6 node (RoutePlannerPipeline) ─────────────────────────────────────────
add_executable(te_6_node
  test/te-6.cpp
)
target_link_libraries(te_6_node
  route_planner_pipeline
)
ament_target_dependencies(te_6_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# ── pybind11 module ───────────────────────────────────────────────────────────
pybind11_add_module(route_planner_py
  src/python/route_planner_py.cpp
)
target_compile_definitions(route_planner_py PRIVATE
  ROUTE_PLANNER_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
)
target_link_libraries(route_planner_py PRIVATE
  route_planner_pipeline
)

install(TARGETS route_planner_py
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)

# ── install ──────────────────────────────────────────────────────────────────
install(TARGETS route_planner_core route_planner_pipeline pointcloud2_adapter pointcloud2_adapter_node pose_adapter pose_adapter_node te_1_node te_2_node te_3_node te_4_node te_5_1_node te_5_2_node te_6_node
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
