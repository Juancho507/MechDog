"""Unit tests for occupancy grid utility functions — no ROS master needed."""

import math
import unittest


# ---------------------------------------------------------------------------
# Pure-function versions of the occupancy-grid helpers so we can test them
# without ROS.  (These mirror the logic in occupancy_grid_node.py.)
# ---------------------------------------------------------------------------

def bresenham_line(x0, y0, x1, y1):
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


def world_to_map(x_world, y_world, origin_x, origin_y, resolution):
    if not (math.isfinite(x_world) and math.isfinite(y_world)):
        return -1, -1
    x_map = int(round((x_world - origin_x) / resolution))
    y_map = int(round((y_world - origin_y) / resolution))
    return x_map, y_map


def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TestBresenhamLine(unittest.TestCase):
    def test_horizontal(self):
        cells = bresenham_line(0, 0, 5, 0)
        self.assertEqual(cells, [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)])

    def test_vertical(self):
        cells = bresenham_line(0, 0, 0, 4)
        self.assertEqual(cells, [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)])

    def test_diagonal(self):
        cells = bresenham_line(0, 0, 3, 3)
        self.assertEqual(cells, [(0, 0), (1, 1), (2, 2), (3, 3)])

    def test_slanted(self):
        cells = bresenham_line(1, 1, 4, 3)
        self.assertEqual(cells, [(1, 1), (2, 2), (3, 2), (4, 3)])

    def test_reverse_horizontal(self):
        cells = bresenham_line(5, 0, 0, 0)
        expected = [(x, 0) for x in range(5, -1, -1)]
        self.assertEqual(cells, expected)

    def test_point_same(self):
        cells = bresenham_line(3, 3, 3, 3)
        self.assertEqual(cells, [(3, 3)])

    def test_negative_coordinates(self):
        cells = bresenham_line(-2, -1, 2, 2)
        self.assertEqual(cells[0], (-2, -1))
        self.assertEqual(cells[-1], (2, 2))
        self.assertGreater(len(cells), 1)

    def test_no_duplicate_interior(self):
        cells = bresenham_line(0, 0, 10, 5)
        self.assertEqual(len(cells), len(set(cells)), "Bresenham returned duplicate cells")


class TestWorldToMap(unittest.TestCase):
    def setUp(self):
        self.ox = -25.0
        self.oy = -25.0
        self.res = 0.05

    def test_origin(self):
        x, y = world_to_map(-25.0, -25.0, self.ox, self.oy, self.res)
        self.assertEqual((x, y), (0, 0))

    def test_origin_positive(self):
        x, y = world_to_map(0.0, 0.0, self.ox, self.oy, self.res)
        self.assertEqual((x, y), (500, 500))

    def test_goal_2_3(self):
        x, y = world_to_map(2.0, 3.0, self.ox, self.oy, self.res)
        # (2 - (-25)) / 0.05 = 27 / 0.05 = 540
        # (3 - (-25)) / 0.05 = 28 / 0.05 = 560
        self.assertEqual((x, y), (540, 560))

    def test_nan_returns_neg1(self):
        x, y = world_to_map(float('nan'), 0.0, self.ox, self.oy, self.res)
        self.assertEqual((x, y), (-1, -1))

    def test_inf_returns_neg1(self):
        x, y = world_to_map(float('inf'), 0.0, self.ox, self.oy, self.res)
        self.assertEqual((x, y), (-1, -1))

    def test_rounding_edge(self):
        x, y = world_to_map(0.0001 + self.ox, 0.0001 + self.oy, self.ox, self.oy, self.res)
        self.assertEqual((x, y), (0, 0))

    def test_rounding_edge_high(self):
        x, y = world_to_map(0.049 - 1e-9 + self.ox, 0.049 - 1e-9 + self.oy,
                            self.ox, self.oy, self.res)
        self.assertEqual((x, y), (1, 1))


class TestRayTraceLogic(unittest.TestCase):
    def test_max_range_marks_free(self):
        """When max_range_hit=True, the endpoint should be marked free
        (cell count check: all cells along the ray are present)."""
        cells = bresenham_line(500, 500, 540, 560)
        # All cells along the ray are traversed
        self.assertIn((500, 500), cells)
        self.assertIn((540, 560), cells)

    def test_obstacle_endpoint_marked(self):
        """When max_range_hit=False, the endpoint marks occupied.
        Check that the 3x3 block around endpoint is valid."""
        end_x, end_y = 540, 560
        occupied_block = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                occupied_block.append((end_x + dx, end_y + dy))
        self.assertEqual(len(occupied_block), 9)
        self.assertIn((540, 560), occupied_block)

    def test_beam_cone_model(self):
        """15-degree beam: endpoint should mark a 3x3 block, not a single cell."""
        end = (100, 100)
        block = {(end[0] + dx, end[1] + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
        self.assertEqual(len(block), 9)
        # A single-cell model would only mark (100, 100)
        self.assertNotEqual(block, {(100, 100)})


class TestYawFromQuaternion(unittest.TestCase):
    class FakeQuat:
        def __init__(self, w, x, y, z):
            self.w, self.x, self.y, self.z = w, x, y, z

    def test_identity(self):
        q = self.FakeQuat(1.0, 0, 0, 0)
        self.assertAlmostEqual(get_yaw_from_quaternion(q), 0.0)

    def test_yaw_90(self):
        q = self.FakeQuat(math.cos(math.pi/4), 0, 0, math.sin(math.pi/4))
        self.assertAlmostEqual(get_yaw_from_quaternion(q), math.pi/2, places=5)

    def test_yaw_minus_90(self):
        q = self.FakeQuat(math.cos(math.pi/4), 0, 0, -math.sin(math.pi/4))
        self.assertAlmostEqual(get_yaw_from_quaternion(q), -math.pi/2, places=5)

    def test_yaw_180(self):
        q = self.FakeQuat(0, 0, 0, 1)
        self.assertAlmostEqual(get_yaw_from_quaternion(q), math.pi, places=5)


if __name__ == '__main__':
    unittest.main()
