# route-planner

LiDAR PointCloud2 → OccupancyGrid → EDT Costmap → A* path planning, exposed to Python via pybind11.

---

## Architecture

<!-- Full pipeline block diagram (T1–T6 thread chain, ROS2 topic I/O, pybind11 interface) -->

---

## Pipeline

`RoutePlannerPipeline` runs as a chain of 6 stages, each on its own thread.

| Stage | Role |
|---|---|
| T1 PointCloud2Adapter | Subscribes `/bridge/sensors/lidar/points` → `PointCloudXYZFrame` |
| T2 PointCloudProcessor | Downsample + range filter |
| T3 PoseAdapter | Subscribes `/bridge/sensors/location` → `PoseXY` |
| T4 OccupancyGridBuilder | PointCloud + Pose → map-aligned 2D grid |
| T5 CostmapBuilder | EDT inflation costmap |
| T6 AStarPlanner | Goal coordinate → path (`list[tuple[float, float]]`) |

---

## Build

```bash
sudo apt-get install -y ros-humble-desktop python3-dev pybind11-dev libyaml-cpp-dev

source /opt/ros/humble/setup.bash
colcon build --packages-select route_planner
```

Output: `install/route_planner/lib/python3.10/site-packages/route_planner_py*.so`

---

## Python API

```python
import route_planner_py as rp

rp.init([])                      # initialize rclcpp (once per process)
rp.ok()                          # → bool

pipeline = rp.RoutePlannerPipeline()
pipeline.start()
pipeline.set_goal(x, y)          # goal in map frame (meters)
path = pipeline.get_path()       # → list[tuple[float, float]]
pipeline.thread_hz()             # → dict[str, float]  per-stage Hz
pipeline.stop()
```

Config files default to the `config/` directory. Pass constructor arguments to override paths.

---

## Config

Five YAML files under `config/` control each stage.

| File | Stage |
|---|---|
| `pointcloud_processor.yaml` | T2 |
| `occupancy_grid_builder.yaml` | T4 |
| `costmap_builder.yaml` | T5 |
| `astar.yaml` | T6 |
| `pipeline.yaml` | Thread rates for the full pipeline |

