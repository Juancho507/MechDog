"""Unit tests for DWA scoring logic — pure functions, no ROS master needed."""

import math
import unittest


# ---------------------------------------------------------------------------
# Pure-function versions of DWA scoring helpers for isolated testing.
# These mirror the logic in local_planner_node.py score_trajectory().
# ---------------------------------------------------------------------------

def score_trajectory(endpoint, cross_track, goal_dist, obstacle_dist,
                     v, w, path_bias=50.0, goal_bias=60.0,
                     occ_scale=15.0, progress_bonus=30.0,
                     speed_bonus=30.0, total_path_len=1.0,
                     robot_pos=(0.0, 0.0), goal_pos=(0.0, 0.0)):
    """Replica of DWA score_trajectory logic as a pure function."""
    path_score = path_bias / (cross_track + 1.0)
    goal_score = goal_bias / (goal_dist + 1.0)

    braking = max(abs(v), 0.05) * 0.5 + 0.1
    if obstacle_dist < braking:
        obstacle_cost = occ_scale * (1.0 - obstacle_dist / braking)
    else:
        obstacle_cost = 0.0

    dx = endpoint[0] - robot_pos[0]
    dy = endpoint[1] - robot_pos[1]
    forward_progress = math.hypot(dx, dy)
    progress = progress_bonus * min(forward_progress / max(total_path_len, 1.0), 0.5)

    # Direction bonus
    gdx = goal_pos[0] - robot_pos[0]
    gdy = goal_pos[1] - robot_pos[1]
    goal_heading = math.atan2(gdy, gdx)
    traj_heading = math.atan2(dy, dx)
    angle_diff = normalize_angle(traj_heading - goal_heading)
    direction_bonus = speed_bonus * v * math.cos(angle_diff)

    score = (path_score + goal_score - obstacle_cost + progress +
             speed_bonus * v + direction_bonus)
    return score


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class TestScoringMonotonicity(unittest.TestCase):
    """Score should improve when the trajectory is better."""

    def test_closer_to_path_is_better(self):
        score_far = score_trajectory(
            (5.0, 2.0), cross_track=2.0, goal_dist=3.0,
            obstacle_dist=10.0, v=0.1, w=0.0)
        score_close = score_trajectory(
            (5.0, 2.0), cross_track=0.5, goal_dist=3.0,
            obstacle_dist=10.0, v=0.1, w=0.0)
        self.assertGreater(score_close, score_far)

    def test_closer_to_goal_is_better(self):
        score_far = score_trajectory(
            (5.0, 2.0), cross_track=1.0, goal_dist=5.0,
            obstacle_dist=10.0, v=0.1, w=0.0)
        score_close = score_trajectory(
            (5.0, 2.0), cross_track=1.0, goal_dist=1.0,
            obstacle_dist=10.0, v=0.1, w=0.0)
        self.assertGreater(score_close, score_far)

    def test_faster_is_better(self):
        score_slow = score_trajectory(
            (5.0, 2.0), cross_track=1.0, goal_dist=3.0,
            obstacle_dist=10.0, v=0.05, w=0.0)
        score_fast = score_trajectory(
            (5.0, 2.0), cross_track=1.0, goal_dist=3.0,
            obstacle_dist=10.0, v=0.2, w=0.0)
        self.assertGreater(score_fast, score_slow)

    def test_facing_goal_is_better(self):
        """Direction bonus rewards trajectories that move toward the goal."""
        robot = (0.0, 0.0)
        goal = (2.0, 0.0)  # goal is to the right
        score_forward = score_trajectory(
            (1.0, 0.0), cross_track=0.5, goal_dist=1.0,
            obstacle_dist=10.0, v=0.1, w=0.0,
            robot_pos=robot, goal_pos=goal)
        score_backward = score_trajectory(
            (-1.0, 0.0), cross_track=0.5, goal_dist=3.0,
            obstacle_dist=10.0, v=0.1, w=0.0,
            robot_pos=robot, goal_pos=goal)
        self.assertGreater(score_forward, score_backward)


class TestScoringEdgeCases(unittest.TestCase):
    def test_zero_cross_track(self):
        """Score must be finite even when trajectory is exactly on path."""
        s = score_trajectory(
            (2.0, 0.0), cross_track=0.0, goal_dist=2.0,
            obstacle_dist=10.0, v=0.1, w=0.0)
        self.assertTrue(math.isfinite(s), f"Score should be finite, got {s}")
        self.assertGreater(s, 0)

    def test_zero_goal_distance(self):
        """Score must be finite when endpoint is exactly at the goal."""
        s = score_trajectory(
            (2.0, 0.0), cross_track=0.5, goal_dist=0.0,
            obstacle_dist=10.0, v=0.1, w=0.0)
        self.assertTrue(math.isfinite(s), f"Score should be finite, got {s}")

    def test_zero_velocity(self):
        """Zero velocity trajectory should still get a score."""
        s = score_trajectory(
            (2.0, 0.0), cross_track=0.5, goal_dist=1.0,
            obstacle_dist=10.0, v=0.0, w=0.0)
        self.assertTrue(math.isfinite(s))

    def test_obstacle_cost_penalty(self):
        """Near obstacle should reduce score compared to far obstacle."""
        score_safe = score_trajectory(
            (2.0, 0.0), cross_track=0.5, goal_dist=1.0,
            obstacle_dist=5.0, v=0.1, w=0.0)
        score_risky = score_trajectory(
            (2.0, 0.0), cross_track=0.5, goal_dist=1.0,
            obstacle_dist=0.05, v=0.1, w=0.0)
        self.assertGreater(score_safe, score_risky)

    def test_negative_velocity(self):
        """Backward velocity (v < 0) should still produce finite score."""
        s = score_trajectory(
            (2.0, 0.0), cross_track=0.5, goal_dist=1.0,
            obstacle_dist=10.0, v=-0.1, w=0.0)
        self.assertTrue(math.isfinite(s))


class TestNormalizeAngle(unittest.TestCase):
    def test_identity(self):
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)

    def test_positive_overflow(self):
        self.assertAlmostEqual(normalize_angle(3.5 * math.pi), -0.5 * math.pi)

    def test_negative_underflow(self):
        self.assertAlmostEqual(normalize_angle(-3.5 * math.pi), 0.5 * math.pi)

    def test_pi(self):
        # pi should stay as pi (or -pi, both are valid)
        n = normalize_angle(math.pi)
        self.assertAlmostEqual(abs(n), math.pi)


class TestDirectionBonus(unittest.TestCase):
    def test_toward_goal(self):
        """cos(0) = 1, direction_bonus = speed_bonus * v."""
        robot = (0.0, 0.0)
        goal = (2.0, 0.0)
        s = score_trajectory(
            (1.0, 0.0), cross_track=0.5, goal_dist=1.0,
            obstacle_dist=10.0, v=0.1, w=0.0,
            robot_pos=robot, goal_pos=goal, speed_bonus=10.0)
        # direction_bonus = 10.0 * 0.1 * cos(0) = 1.0
        # 1/(0.5+1) = 0.667, 1/(1+1) = 0.5, progress, speed, direction
        self.assertTrue(math.isfinite(s))

    def test_away_from_goal(self):
        """cos(pi) = -1, direction_bonus = -speed_bonus * v (penalty)."""
        robot = (0.0, 0.0)
        goal = (2.0, 0.0)
        s = score_trajectory(
            (-1.0, 0.0), cross_track=0.5, goal_dist=3.0,
            obstacle_dist=10.0, v=0.1, w=0.0,
            robot_pos=robot, goal_pos=goal, speed_bonus=10.0)
        # direction_bonus = 10.0 * 0.1 * cos(pi) = -1.0
        self.assertTrue(math.isfinite(s))
        # Moving away should always score lower than moving toward
        s_toward = score_trajectory(
            (1.0, 0.0), cross_track=0.5, goal_dist=1.0,
            obstacle_dist=10.0, v=0.1, w=0.0,
            robot_pos=robot, goal_pos=goal)
        self.assertGreater(s_toward, s)


if __name__ == '__main__':
    unittest.main()
