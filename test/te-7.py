# [TE-7] pybind11 바인딩 검증
#
# 목적:
#   route_planner_py 모듈을 통해 RoutePlannerPipeline을 Python에서
#   직접 제어할 수 있는지 검증한다.
#   - rp.init() / rp.shutdown() 동작
#   - 파이프라인 생성 (기본 config 경로)
#   - set_goal() / get_path() API
#   - 5초마다 스레드 Hz + path 상태 출력
#
# 실행:
#   source env.sh && source install/setup.bash
#   python3 test/te-7.py \
#     --ros-args --params-file config/pointcloud2_adapter.yaml \
#                --params-file config/pose_adapter.yaml \
#     -- --goal_x 5.17 --goal_y 0.7
#
# 옵션 (-- 뒤에):
#   --goal_x FLOAT   목표 x 좌표 (기본값: 2.0)
#   --goal_y FLOAT   목표 y 좌표 (기본값: 0.0)

import argparse
import signal
import sys
import time

import route_planner_py as rp


def parse_goal_args() -> tuple[float, float]:
    """-- 뒤의 인자만 파싱한다 (--ros-args 블록과 충돌 방지)."""
    try:
        sep = sys.argv.index("--")
        own_args = sys.argv[sep + 1:]
    except ValueError:
        own_args = []

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--goal_x", type=float, default=2.0)
    parser.add_argument("--goal_y", type=float, default=0.0)
    args, _ = parser.parse_known_args(own_args)
    return args.goal_x, args.goal_y


def main() -> None:
    goal_x, goal_y = parse_goal_args()

    rp.init(sys.argv)
    print(f"[te-7] rclcpp initialized")

    pipeline = rp.RoutePlannerPipeline()
    pipeline.set_goal(goal_x, goal_y)
    pipeline.start()
    print(f"[te-7] pipeline started  goal=({goal_x:.2f}, {goal_y:.2f})")

    stop = False

    def _on_sigint(sig, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _on_sigint)

    last_print = time.monotonic() - 5.0   # 첫 tick에서 바로 출력

    while not stop:
        now = time.monotonic()
        if now - last_print >= 5.0:
            hz   = pipeline.thread_hz()
            path = pipeline.get_path()
            print(
                f"[Hz]  T1={hz['t1_pointcloud2_spin']:.1f}"
                f"  T2={hz['t2_pose_spin']:.1f}"
                f"  T3={hz['t3_processor']:.1f}"
                f"  T4={hz['t4_occupancy_grid']:.1f}"
                f"  T5={hz['t5_costmap']:.1f}"
                f"  T6={hz['t6_astar']:.1f}"
            )
            if path:
                print(f"[path] {len(path)} waypoints  "
                      f"start=({path[0][0]:.2f}, {path[0][1]:.2f})  "
                      f"end=({path[-1][0]:.2f}, {path[-1][1]:.2f})")
            else:
                print("[path] no path yet")
            last_print = now

        time.sleep(0.1)

    print("[te-7] stopping...")
    pipeline.stop()
    rp.shutdown()
    print("[te-7] done")


if __name__ == "__main__":
    main()
