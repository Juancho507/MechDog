#!/usr/bin/env python
"""
AlgorithmBenchmark — Automated path-planning benchmark for MechDog.
Loads a standard test map, runs A*, Dijkstra, and BFS on the same
PlanningProblem, logs results to stdout and appends to results.csv.

Usage:
    # Standalone (requires a ROS bag or pre-recorded map, or synthetic test):
    python experiment_runner.py

    # With live ROS (uses current /mechdog/map):
    python experiment_runner.py --live
"""

import sys
import os
import time
import csv
import argparse

# Ensure catkin workspace Python path
_ws_src = os.path.join(os.path.dirname(__file__), 'catkin_ws', 'src')
if _ws_src not in sys.path:
    sys.path.insert(0, _ws_src)

from mechdog_navigation.scripts.planner_strategy import (
    AStarPlanner, DijkstraPlanner, BFSPlanner,
    PlanningProblem, PlanningResult,
)
from nav_msgs.msg import OccupancyGrid

# ---------------------------------------------------------------------------
#  Synthetic test map  (100×100, open corridor with a few obstacles)
# ---------------------------------------------------------------------------

def _build_test_map(width=100, height=100, resolution=0.1) -> OccupancyGrid:
    """50 m × 50 m map, clear except a wall across the middle with a gap."""
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.position.x = -25.0
    grid.info.origin.position.y = -25.0
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0

    data = [0] * (width * height)

    # Wall across y = 50 (middle), gap at x = 40..60
    for x in range(width):
        for y in [49, 50, 51]:
            if 40 <= x <= 60:
                continue  # gap
            idx = y * width + x
            data[idx] = 100

    # A few pillars
    pillars = [(20, 20), (70, 30), (30, 70)]
    for px, py in pillars:
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                if dx * dx + dy * dy > 9:
                    continue
                cx, cy = px + dx, py + dy
                if 0 <= cx < width and 0 <= cy < height:
                    data[cy * width + cx] = 100

    grid.data = data
    return grid


# ---------------------------------------------------------------------------
#  Benchmark runner
# ---------------------------------------------------------------------------

_ALGORITHMS = {
    'A*':       AStarPlanner,
    'Dijkstra': DijkstraPlanner,
    'BFS':      BFSPlanner,
}

_RESULTS_FILE = os.path.join(os.path.dirname(__file__), 'results.csv')
_FIELDS = [
    'algorithm', 'success', 'nodes_expanded', 'cpu_time_ms',
    'path_length_cells', 'path_length_metres',
    'start_x', 'start_y', 'goal_x', 'goal_y',
]


class AlgorithmBenchmark:
    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid

    def run_goal(self, start_world, goal_world, label="test") -> dict:
        """Run all planners on one (start → goal). Returns row dicts."""
        res = self.grid.info.resolution
        ox = self.grid.info.origin.position.x
        oy = self.grid.info.origin.position.y

        def to_grid(p):
            return (int((p[0] - ox) / res), int((p[1] - oy) / res))

        problem = PlanningProblem(
            start_grid=to_grid(start_world),
            goal_grid=to_grid(goal_world),
            occupancy_grid=self.grid,
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
                'algorithm':        name,
                'success':          int(result.success),
                'nodes_expanded':   result.nodes_expanded,
                'cpu_time_ms':      round(result.cpu_time_ms, 2),
                'path_length_cells': result.path_length_cells,
                'path_length_metres': round(result.path_length_metres, 4),
                'start_x':          start_world[0],
                'start_y':          start_world[1],
                'goal_x':           goal_world[0],
                'goal_y':           goal_world[1],
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
        """Append rows to results.csv."""
        exists = os.path.isfile(_RESULTS_FILE)
        with open(_RESULTS_FILE, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=_FIELDS)
            if not exists:
                writer.writeheader()
            writer.writerows(rows)
        print(f"\n  → Appended {len(rows)} rows to {_RESULTS_FILE}")

    def run_suite(self):
        """Run a standard set of start→goal tests."""
        tests = [
            ((0, 0), (4, 0),    "straight forward"),
            ((0, 0), (-4, 0),   "straight backward"),
            ((0, 0), (0, 4),    "straight left"),
            ((0, 0), (0, -4),   "straight right"),
            ((0, 0), (5, 5),    "diagonal"),
            ((-3, 0), (3, 0),   "cross wall with gap"),
            ((0, -2), (0, 2),   "cross wall L→R through gap"),
        ]

        for start, goal, desc in tests:
            print(f"\n  Goal: {desc}  ({start} → {goal})")
            rows = self.run_goal(start, goal, desc)
            self.write_csv(rows)


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

    print("\nDone. Results saved to results.csv")


if __name__ == '__main__':
    main()
