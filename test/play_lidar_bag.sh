#!/bin/bash
# bag 파일의 /point_cloud 토픽을 /bridge/sensors/lidar/points 로 리매핑해서 재생

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_PATH="$SCRIPT_DIR/bags/rosbag2_2022_04_14-ped_vehicle"

if [ ! -d "$BAG_PATH" ]; then
    echo "Error: bag not found at $BAG_PATH"
    exit 1
fi

ros2 bag play "$BAG_PATH" \
    --remap /sensing/lidar/top/rectified/pointcloud:=/bridge/sensors/lidar/points \
    --loop
