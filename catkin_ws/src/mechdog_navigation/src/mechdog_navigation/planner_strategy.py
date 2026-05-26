#!/usr/bin/env python
"""
PlannerStrategy — Abstract base class for path planning algorithms (A*, Dijkstra, BFS).
All implementations share the same interface so they can be benchmarked identically.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from nav_msgs.msg import OccupancyGrid


@dataclass
class PlanningProblem:
    start_grid: Tuple[int, int]
    goal_grid: Tuple[int, int]
    occupancy_grid: OccupancyGrid
    inflation_radius: int = 3    # cells to inflate around obstacles
    goal_tolerance: int = 2      # cells
    cell_size: float = 0.1       # metres per cell


@dataclass
class PlanningResult:
    path: List[Tuple[int, int]]         # grid cells from start → goal
    nodes_expanded: int = 0
    cpu_time_ms: float = 0.0
    path_length_cells: int = 0
    path_length_metres: float = 0.0
    success: bool = False
    algorithm_name: str = ""


class PlannerStrategy(ABC):
    @abstractmethod
    def plan(self, problem: PlanningProblem) -> PlanningResult:
        ...


# ---------------------------------------------------------------------------
#  Common helpers
# ---------------------------------------------------------------------------

def _inflate_map(grid: OccupancyGrid, radius: int) -> OccupancyGrid:
    """Apply binary dilation (inflation) around occupied cells."""
    import copy
    import numpy as np
    w, h = grid.info.width, grid.info.height
    arr = np.array(grid.data, dtype=np.int8).reshape((h, w))
    inflated = arr.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            shifted = np.roll(np.roll(arr, dy, axis=0), dx, axis=1)
            mask = shifted > 50
            if dy < 0:
                mask[dy:, :] = False
            elif dy > 0:
                mask[:dy, :] = False
            if dx < 0:
                mask[:, dx:] = False
            elif dx > 0:
                mask[:, :dx] = False
            inflated[mask] = 100
    result = copy.deepcopy(grid)
    result.data = inflated.flatten().tolist()
    return result


def _is_occupied(cell, grid: OccupancyGrid) -> bool:
    x, y = cell
    idx = y * grid.info.width + x
    return grid.data[idx] > 50


def _is_valid(cell, grid: OccupancyGrid) -> bool:
    x, y = cell
    return 0 <= x < grid.info.width and 0 <= y < grid.info.height


def _manhattan(a, b) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _euclidean_sq(a, b) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def _reconstruct(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# ---------------------------------------------------------------------------
#  A*
# ---------------------------------------------------------------------------

class AStarPlanner(PlannerStrategy):
    def plan(self, problem: PlanningProblem) -> PlanningResult:
        import heapq, time
        t0 = time.perf_counter()
        grid = _inflate_map(problem.occupancy_grid, problem.inflation_radius)
        start, goal = problem.start_grid, problem.goal_grid
        if not _is_valid(start, grid) or not _is_valid(goal, grid):
            return PlanningResult(success=False, algorithm_name="A*", path=[])

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        nodes = 0

        while open_set:
            current = heapq.heappop(open_set)[1]
            nodes += 1
            if _euclidean_sq(current, goal) <= problem.goal_tolerance ** 2:
                dt = (time.perf_counter() - t0) * 1000
                path = _reconstruct(came_from, current)
                return PlanningResult(
                    path=path, nodes_expanded=nodes, cpu_time_ms=dt,
                    path_length_cells=len(path),
                    path_length_metres=len(path) * problem.cell_size,
                    success=True, algorithm_name="A*")

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nb = (current[0] + dx, current[1] + dy)
                if not _is_valid(nb, grid) or _is_occupied(nb, grid):
                    continue
                tg = g_score[current] + 1
                if nb not in g_score or tg < g_score[nb]:
                    came_from[nb] = current
                    g_score[nb] = tg
                    f = tg + _manhattan(nb, goal)
                    heapq.heappush(open_set, (f, nb))

        dt = (time.perf_counter() - t0) * 1000
        return PlanningResult(success=False, algorithm_name="A*", path=[],
                              nodes_expanded=nodes, cpu_time_ms=dt)


# ---------------------------------------------------------------------------
#  Dijkstra (uniform-cost search)
# ---------------------------------------------------------------------------

class DijkstraPlanner(PlannerStrategy):
    def plan(self, problem: PlanningProblem) -> PlanningResult:
        import heapq, time
        t0 = time.perf_counter()
        grid = _inflate_map(problem.occupancy_grid, problem.inflation_radius)
        start, goal = problem.start_grid, problem.goal_grid
        if not _is_valid(start, grid) or not _is_valid(goal, grid):
            return PlanningResult(success=False, algorithm_name="Dijkstra", path=[])

        open_set = [(0, start)]
        came_from = {}
        dist = {start: 0}
        nodes = 0

        while open_set:
            current = heapq.heappop(open_set)[1]
            nodes += 1
            if _euclidean_sq(current, goal) <= problem.goal_tolerance ** 2:
                dt = (time.perf_counter() - t0) * 1000
                path = _reconstruct(came_from, current)
                return PlanningResult(
                    path=path, nodes_expanded=nodes, cpu_time_ms=dt,
                    path_length_cells=len(path),
                    path_length_metres=len(path) * problem.cell_size,
                    success=True, algorithm_name="Dijkstra")

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nb = (current[0] + dx, current[1] + dy)
                if not _is_valid(nb, grid) or _is_occupied(nb, grid):
                    continue
                nd = dist[current] + 1
                if nb not in dist or nd < dist[nb]:
                    came_from[nb] = current
                    dist[nb] = nd
                    heapq.heappush(open_set, (nd, nb))

        dt = (time.perf_counter() - t0) * 1000
        return PlanningResult(success=False, algorithm_name="Dijkstra", path=[],
                              nodes_expanded=nodes, cpu_time_ms=dt)


# ---------------------------------------------------------------------------
#  BFS
# ---------------------------------------------------------------------------

class BFSPlanner(PlannerStrategy):
    def plan(self, problem: PlanningProblem) -> PlanningResult:
        from collections import deque
        import time
        t0 = time.perf_counter()
        grid = _inflate_map(problem.occupancy_grid, problem.inflation_radius)
        start, goal = problem.start_grid, problem.goal_grid
        if not _is_valid(start, grid) or not _is_valid(goal, grid):
            return PlanningResult(success=False, algorithm_name="BFS", path=[])

        queue = deque([[start]])
        visited = {start}
        nodes = 0

        while queue:
            path = queue.popleft()
            current = path[-1]
            nodes += 1
            if _euclidean_sq(current, goal) <= problem.goal_tolerance ** 2:
                dt = (time.perf_counter() - t0) * 1000
                return PlanningResult(
                    path=path, nodes_expanded=nodes, cpu_time_ms=dt,
                    path_length_cells=len(path),
                    path_length_metres=len(path) * problem.cell_size,
                    success=True, algorithm_name="BFS")

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nb = (current[0] + dx, current[1] + dy)
                if not _is_valid(nb, grid) or _is_occupied(nb, grid):
                    continue
                if nb in visited:
                    continue
                visited.add(nb)
                queue.append(path + [nb])

        dt = (time.perf_counter() - t0) * 1000
        return PlanningResult(success=False, algorithm_name="BFS", path=[],
                              nodes_expanded=nodes, cpu_time_ms=dt)
