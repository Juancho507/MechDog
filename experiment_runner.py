#!/usr/bin/env python
"""
AlgorithmBenchmark — Automated path-planning benchmark for MechDog.
Two scenarios:
  A (Static) … immediate path calculation without rotation.
  B (Active) … simulates scan behavior (360° rotation) then calculates path.

Usage:
    python experiment_runner.py
    python experiment_runner.py --live
"""

import sys
import os
import time
import csv
import argparse
import math
import copy

_ws_scripts = os.path.join(os.path.dirname(__file__), 'catkin_ws', 'src', 'mechdog_navigation', 'scripts')
if _ws_scripts not in sys.path:
    sys.path.insert(0, _ws_scripts)

from planner_strategy import (
    AStarPlanner, DijkstraPlanner, BFSPlanner,
    PlanningProblem, PlanningResult,
)
from nav_msgs.msg import OccupancyGrid


# ---------------------------------------------------------------------------
#  Synthetic test map  (100×100, open corridor with a few obstacles)
# ---------------------------------------------------------------------------

def _build_test_map(width=500, height=500, resolution=0.1) -> OccupancyGrid:
    """50 m × 50 m map, wall across the middle with a gap, plus pillars.
    Origin at (-25, -25) gives valid world coords from -25 to +25."""
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.position.x = -25.0
    grid.info.origin.position.y = -25.0
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0

    data = [0] * (width * height)
    mid = height // 2  # 250

    # Wall across y = mid, gap in x range [mid - 60, mid + 60]
    for x in range(width):
        for y in [mid - 1, mid, mid + 1]:
            if mid - 60 <= x <= mid + 60:
                continue
            idx = y * width + x
            data[idx] = 100

    # Pillars at grid cells — avoid center (robot start area)
    pillars_px = [
        (mid + 100, mid - 50),   # world (10, -5)
        (mid - 80, mid - 30),    # world (-8, -3)
        (mid + 40, mid + 100),   # world (4, 10)
    ]
    for px, py in pillars_px:
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                if dx * dx + dy * dy > 9:
                    continue
                cx, cy = px + dx, py + dy
                if 0 <= cx < width and 0 <= cy < height:
                    data[cy * width + cx] = 100

    grid.data = data
    return grid


def _simulate_scan_behavior(grid: OccupancyGrid, robot_world=(0.0, 0.0), radius_m=2.0) -> OccupancyGrid:
    """Simulate the Scan Behavior: add free-space observations in a circle
    around the robot, as if it had rotated 360° with its ultrasonic sensor."""
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    r_cells = int(round(radius_m / res))

    robot_gx = int(round((robot_world[0] - ox) / res))
    robot_gy = int(round((robot_world[1] - oy) / res))

    result = copy.deepcopy(grid)
    data = list(result.data)
    w = result.info.width
    h = result.info.height

    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            if dx * dx + dy * dy > r_cells * r_cells:
                continue
            gx = robot_gx + dx
            gy = robot_gy + dy
            if 0 <= gx < w and 0 <= gy < h:
                idx = gy * w + gx
                if data[idx] != 100:
                    data[idx] = 0

    result.data = data
    return result


# ---------------------------------------------------------------------------
#  Benchmark runner
# ---------------------------------------------------------------------------

_ALGORITHMS = {
    'A*':       AStarPlanner,
    'Dijkstra': DijkstraPlanner,
    'BFS':      BFSPlanner,
}

_RESULTS_FILE = os.path.join(os.path.dirname(__file__), 'benchmark_results.csv')
_FIELDS = [
    'scenario', 'algorithm', 'success', 'nodes_expanded', 'cpu_time_ms',
    'path_length_cells', 'path_length_metres',
    'start_x', 'start_y', 'goal_x', 'goal_y',
]


class AlgorithmBenchmark:
    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid

    def run_goal(self, start_world, goal_world, label="test", scenario="A") -> list:
        res = self.grid.info.resolution
        ox = self.grid.info.origin.position.x
        oy = self.grid.info.origin.position.y

        def to_grid(p):
            return (int(round((p[0] - ox) / res)), int(round((p[1] - oy) / res)))

        if scenario == 'B':
            scan_grid = _simulate_scan_behavior(self.grid, start_world, radius_m=2.0)
        else:
            scan_grid = self.grid

        problem = PlanningProblem(
            start_grid=to_grid(start_world),
            goal_grid=to_grid(goal_world),
            occupancy_grid=scan_grid,
            inflation_radius=3,
            goal_tolerance=2,
            cell_size=res,
        )

        rows = []
        for name, cls in _ALGORITHMS.items():
            t0 = time.perf_counter()
            result: PlanningResult = cls().plan(problem)
            elapsed = (time.perf_counter() - t0) * 1000

            row = {
                'scenario':          f'Scenario_{scenario}',
                'algorithm':         name,
                'success':           int(result.success),
                'nodes_expanded':    result.nodes_expanded,
                'cpu_time_ms':       round(result.cpu_time_ms, 2),
                'path_length_cells': result.path_length_cells,
                'path_length_metres': round(result.path_length_metres, 4),
                'start_x':           start_world[0],
                'start_y':           start_world[1],
                'goal_x':            goal_world[0],
                'goal_y':            goal_world[1],
            }
            rows.append(row)

            status = "OK" if result.success else "FAIL"
            print(f"  [{status}] {name:>8s}  "
                  f"nodes={result.nodes_expanded:>5d}  "
                  f"time={result.cpu_time_ms:>8.2f} ms  "
                  f"path={result.path_length_metres:>6.2f} m  "
                  f"(wall={elapsed:>8.2f} ms)")

        return rows

    def write_csv(self, rows):
        exists = os.path.isfile(_RESULTS_FILE)
        with open(_RESULTS_FILE, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=_FIELDS)
            if not exists:
                writer.writeheader()
            writer.writerows(rows)
        print(f"  → Appended {len(rows)} rows to {_RESULTS_FILE}")

    def run_suite(self):
        tests = [
            ((0, 0), (4, 0),    "straight forward"),
            ((0, 0), (-4, 0),   "straight backward"),
            ((0, 0), (0, 4),    "straight left"),
            ((0, 0), (0, -4),   "straight right"),
            ((0, 0), (5, 5),    "diagonal"),
            ((-3, 0), (3, 0),   "cross wall with gap"),
            ((0, -2), (0, 2),   "cross wall L→R through gap"),
        ]

        for scenario in ('A', 'B'):
            label = "Static (no scan)" if scenario == 'A' else "Active (with scan)"
            print(f"\n{'='*60}")
            print(f"  Scenario {scenario}: {label}")
            print(f"{'='*60}")
            for start, goal, desc in tests:
                print(f"\n  Goal: {desc}  ({start} → {goal})")
                rows = self.run_goal(start, goal, desc, scenario)
                self.write_csv(rows)

        print(f"\n  Summary: benchmark_results.csv contains both scenarios A and B")


# ---------------------------------------------------------------------------
#  CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="MechDog Algorithm Benchmark")
    parser.add_argument('--live', action='store_true',
                        help='Use current ROS /mechdog/map instead of synthetic test map')
    args = parser.parse_args()

    if args.live:
        try:
            import rospy
            rospy.init_node('algorithm_benchmark', anonymous=True)
            msg = rospy.wait_for_message('/mechdog/map', OccupancyGrid, timeout=10)
            grid = msg
            print("Loaded live map from /mechdog/map")
        except Exception as e:
            print(f"Failed to get live map: {e}")
            print("Falling back to synthetic test map.")
            grid = _build_test_map()
    else:
        grid = _build_test_map()
        print(f"Using synthetic test map ({grid.info.width}×{grid.info.height} @ {grid.info.resolution} m)")

    benchmark = AlgorithmBenchmark(grid)
    benchmark.run_suite()

    print("\nDone. Results saved to benchmark_results.csv")


if __name__ == '__main__':
    main()
