"""Unit tests for PlannerStrategy (A*, Dijkstra, BFS) — no ROS dependency."""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import math
import unittest
from mechdog_navigation.planner_strategy import (
    AStarPlanner, DijkstraPlanner, BFSPlanner,
    PlanningProblem, PlanningResult,
    _inflate_map, _is_occupied, _is_valid,
    _manhattan, _euclidean_sq, _reconstruct,
)
from nav_msgs.msg import OccupancyGrid


def _make_grid(width, height, data=None, resolution=0.1):
    """Helper: create an OccupancyGrid with optional flat data."""
    g = OccupancyGrid()
    g.info.width = width
    g.info.height = height
    g.info.resolution = resolution
    g.info.origin.position.x = -width * resolution / 2
    g.info.origin.position.y = -height * resolution / 2
    if data is None:
        data = [0] * (width * height)
    g.data = data
    return g


class TestHelpers(unittest.TestCase):
    def test_manhattan(self):
        self.assertEqual(_manhattan((0, 0), (3, 4)), 7)

    def test_euclidean_sq(self):
        self.assertEqual(_euclidean_sq((0, 0), (3, 4)), 25)

    def test_is_valid_in_bounds(self):
        g = _make_grid(10, 10)
        self.assertTrue(_is_valid((5, 5), g))
        self.assertTrue(_is_valid((0, 0), g))
        self.assertTrue(_is_valid((9, 9), g))

    def test_is_valid_out_of_bounds(self):
        g = _make_grid(10, 10)
        self.assertFalse(_is_valid((-1, 5), g))
        self.assertFalse(_is_valid((5, -1), g))
        self.assertFalse(_is_valid((10, 5), g))
        self.assertFalse(_is_valid((5, 10), g))

    def test_is_occupied_free(self):
        g = _make_grid(10, 10)
        self.assertFalse(_is_occupied((5, 5), g))

    def test_is_occupied_occupied(self):
        data = [0] * 100
        data[5 * 10 + 5] = 100
        g = _make_grid(10, 10, data)
        self.assertTrue(_is_occupied((5, 5), g))

    def test_is_occupied_boundary(self):
        data = [0] * 100
        data[5 * 10 + 5] = 50
        g = _make_grid(10, 10, data)
        self.assertFalse(_is_occupied((5, 5), g))

    def test_reconstruct(self):
        came_from = {(1, 0): (0, 0), (2, 0): (1, 0), (3, 0): (2, 0)}
        path = _reconstruct(came_from, (3, 0))
        self.assertEqual(path, [(0, 0), (1, 0), (2, 0), (3, 0)])


class TestInflateMap(unittest.TestCase):
    def test_no_inflation(self):
        data = [0] * 100
        data[5 * 10 + 5] = 100
        g = _make_grid(10, 10, data)
        inflated = _inflate_map(g, 0)
        self.assertEqual(inflated.data[5 * 10 + 5], 100)

    def test_inflation_expands_occupied(self):
        data = [0] * 100
        data[5 * 10 + 5] = 100
        g = _make_grid(10, 10, data)
        inflated = _inflate_map(g, 1)
        # Center cell remains occupied
        self.assertEqual(inflated.data[5 * 10 + 5], 100)
        # Immediate neighbors also occupied
        self.assertEqual(inflated.data[4 * 10 + 5], 100)  # above
        self.assertEqual(inflated.data[6 * 10 + 5], 100)  # below
        self.assertEqual(inflated.data[5 * 10 + 4], 100)  # left
        self.assertEqual(inflated.data[5 * 10 + 6], 100)  # right
        # Diagonal neighbor (1,1) is outside circular radius 1: 1²+1²=2 > 1
        self.assertEqual(inflated.data[4 * 10 + 4], 0)

    def test_inflation_radius_circle(self):
        data = [0] * 100
        data[5 * 10 + 5] = 100
        g = _make_grid(10, 10, data)
        inflated = _inflate_map(g, 3)
        # Within radius 3: cell (5, 5 ± 2)
        self.assertEqual(inflated.data[3 * 10 + 5], 100)
        # Outside radius 3: cell (5, 5 ± 3) — exactly on boundary, Pythagorean says yes if <=3
        # but (0,3) has 0^2 + 3^2 = 9 <= 9 -> within radius
        self.assertEqual(inflated.data[2 * 10 + 5], 100)


class PlannerTestBase:
    """Mixin with shared test cases for all planners."""

    def _run(self, grid, start, goal, **kw):
        problem = PlanningProblem(
            start_grid=start, goal_grid=goal,
            occupancy_grid=grid, **kw)
        return self.planner.plan(problem)

    def test_empty_grid_straight_line(self):
        grid = _make_grid(20, 20)
        result = self._run(grid, (2, 10), (17, 10))
        self.assertTrue(result.success, f"{self.planner.__class__.__name__} failed on empty grid")
        self.assertEqual(result.path[0], (2, 10))
        self.assertLessEqual(
            _euclidean_sq(result.path[-1], (17, 10)),
            self.planner.plan.__globals__.get('goal_tolerance', 2) ** 2)

    def test_obstacle_avoids_blocked_direct_path(self):
        """Direct line is blocked; path must go around."""
        data = [0] * 400
        for y in range(5, 15):
            for x in range(7, 12):
                data[y * 20 + x] = 100
        grid = _make_grid(20, 20, data)
        result = self._run(grid, (2, 10), (17, 10))
        self.assertTrue(result.success, f"{self.planner.__class__.__name__} blocked by obstacle")
        # Path must not go through any occupied cell
        for cx, cy in result.path:
            self.assertFalse(_is_occupied((cx, cy), grid),
                             f"Path crosses obstacle at ({cx}, {cy})")

    def test_obstacle_navigates_around(self):
        """Path should deviate around a centered obstacle."""
        data = [0] * 400
        for dy in (-2, -1, 0, 1, 2):
            for dx in (-2, -1, 0, 1, 2):
                data[(10 + dy) * 20 + (10 + dx)] = 100
        grid = _make_grid(20, 20, data)
        result = self._run(grid, (2, 10), (17, 10))
        self.assertTrue(result.success, f"{self.planner.__class__.__name__} failed around obstacle")
        # Path should have reasonable length (not trivial)
        self.assertGreater(len(result.path), 10)

    def test_inflation_creates_clearance(self):
        """With inflation, path should maintain distance from obstacle."""
        data = [0] * 400
        data[10 * 20 + 10] = 100
        grid = _make_grid(20, 20, data)
        result = self._run(grid, (2, 10), (17, 10), inflation_radius=3)
        self.assertTrue(result.success)
        # No cell should be within 3 cells of the obstacle
        for cx, cy in result.path:
            d = _euclidean_sq((cx, cy), (10, 10))
            self.assertGreater(d, 9, f"Path too close to obstacle at ({cx}, {cy})")

    def test_unreachable_goal(self):
        """Goal completely surrounded by obstacles returns failure."""
        data = [0] * 100
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                data[(5 + dy) * 10 + (5 + dx)] = 100
        grid = _make_grid(10, 10, data)
        result = self._run(grid, (1, 5), (5, 5))
        self.assertFalse(result.success,
                         f"{self.planner.__class__.__name__} should fail on unreachable goal")

    def test_start_is_goal(self):
        """Start equals goal => immediate success with single-cell path."""
        grid = _make_grid(10, 10)
        result = self._run(grid, (5, 5), (5, 5))
        self.assertTrue(result.success)
        self.assertEqual(len(result.path), 1)

    def test_goal_tolerance(self):
        """Path should end within tolerance of the goal cell, not necessarily at it."""
        grid = _make_grid(10, 10)
        result = self._run(grid, (0, 0), (9, 9), goal_tolerance=2)
        self.assertTrue(result.success)
        last = result.path[-1]
        self.assertLessEqual(_euclidean_sq(last, (9, 9)), 4)

    def test_corner_to_corner(self):
        """Diagonal traversal across whole grid."""
        grid = _make_grid(30, 30)
        result = self._run(grid, (0, 0), (29, 29))
        self.assertTrue(result.success)
        self.assertGreater(len(result.path), 20)


class TestAStar(unittest.TestCase, PlannerTestBase):
    def setUp(self):
        self.planner = AStarPlanner()


class TestDijkstra(unittest.TestCase, PlannerTestBase):
    def setUp(self):
        self.planner = DijkstraPlanner()


class TestBFS(unittest.TestCase, PlannerTestBase):
    def setUp(self):
        self.planner = BFSPlanner()


class TestResultDataclass(unittest.TestCase):
    def test_defaults(self):
        r = PlanningResult(path=[], algorithm_name="test")
        self.assertFalse(r.success)
        self.assertEqual(r.nodes_expanded, 0)
        self.assertEqual(r.cpu_time_ms, 0.0)

    def test_success(self):
        r = PlanningResult(path=[(0, 0), (1, 0)], success=True, algorithm_name="test",
                           path_length_cells=2)
        self.assertTrue(r.success)
        self.assertEqual(r.path_length_cells, 2)


if __name__ == '__main__':
    unittest.main()
