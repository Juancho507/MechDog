#!/usr/bin/env python
"""
Global Planner Node for MechDog Navigation
Delegates to PlannerStrategy implementations (A*, Dijkstra, BFS)
so the ROS node remains a thin wrapper for benchmarking.
100 % portable — no simulation dependencies.
"""
import rospy
import numpy as np
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from mechdog_navigation.planner_strategy import (
    PlanningProblem, PlanningResult,
    AStarPlanner, DijkstraPlanner, BFSPlanner,
)


_ALGORITHM_MAP = {
    'astar':    AStarPlanner,
    'dijkstra': DijkstraPlanner,
    'bfs':      BFSPlanner,
}


class GlobalPlanner:
    def __init__(self):
        rospy.init_node('global_planner', anonymous=False)
        self.load_parameters()

        self.current_map = None
        self.current_goal = None
        self.robot_grid = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self.path_pub = rospy.Publisher(
            self.param_plan_output, Path, queue_size=1)
        self.map_sub = rospy.Subscriber(
            self.param_map_input, OccupancyGrid, self.map_callback)
        self.goal_sub = rospy.Subscriber(
            self.param_goal_input, PoseStamped, self.goal_callback)

        self.replan_timer = rospy.Timer(
            rospy.Duration(1.0 / self.param_planning_frequency),
            self.replan_callback)

        rospy.loginfo("Global Planner initialized — algorithm: %s", self.param_algorithm)

    def load_parameters(self):
        self.param_algorithm = rospy.get_param('~global_planner/algorithm', 'astar')
        self.param_cell_size = rospy.get_param('~global_planner/planning/cell_size', 0.1)
        self.param_planning_frequency = rospy.get_param(
            'navigation/planning/global_planning_frequency', 1.0)
        self.param_inflation_cells = rospy.get_param(
            '~global_planner/inflation/cells', 3)
        self.param_goal_tolerance_cells = rospy.get_param(
            '~global_planner/goal/tolerance_cells', 2)

        self.param_map_input = rospy.get_param(
            '~global_planner/topics/map_input', '/mechdog/map')
        self.param_goal_input = rospy.get_param(
            '~global_planner/topics/goal_input', '/mechdog/goal')
        self.param_plan_output = rospy.get_param(
            '~global_planner/topics/plan_output', '/mechdog/global_plan')
        self.param_global_frame = rospy.get_param(
            '~global_planner/frames/global_frame', 'map')

    def map_callback(self, msg):
        self.current_map = msg

    def goal_callback(self, msg):
        self.current_goal = msg
        self.plan_path()

    def replan_callback(self, event):
        if self.current_goal is not None and self.current_map is not None:
            self.plan_path()

    def _get_robot_grid(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self.param_global_frame, 'base_footprint', rospy.Time(0),
                rospy.Duration(0.5))
            return self.world_to_grid(
                t.transform.translation.x,
                t.transform.translation.y)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Cannot get robot pose from TF: %s", e)
            return self.world_to_grid(0.0, 0.0)

    def plan_path(self):
        if self.current_map is None or self.current_goal is None:
            return

        start_grid = self._get_robot_grid()
        goal_grid = self.world_to_grid(
            self.current_goal.pose.position.x,
            self.current_goal.pose.position.y)

        problem = PlanningProblem(
            start_grid=start_grid,
            goal_grid=goal_grid,
            occupancy_grid=self.current_map,
            inflation_radius=self.param_inflation_cells,
            goal_tolerance=self.param_goal_tolerance_cells,
            cell_size=self.param_cell_size,
        )

        planner_cls = _ALGORITHM_MAP.get(self.param_algorithm)
        if planner_cls is None:
            rospy.logwarn("Unknown algorithm %s, falling back to A*", self.param_algorithm)
            planner_cls = AStarPlanner

        result: PlanningResult = planner_cls().plan(problem)

        if result.success:
            path_msg = self.grid_path_to_msg(result.path)
            self.path_pub.publish(path_msg)
            rospy.loginfo(
                "%s — path found: %d waypoints, %d nodes expanded, %.2f ms",
                result.algorithm_name, result.path_length_cells,
                result.nodes_expanded, result.cpu_time_ms)
        else:
            rospy.logwarn(
                "%s — no path found (%d nodes, %.2f ms)",
                result.algorithm_name, result.nodes_expanded, result.cpu_time_ms)

    def world_to_grid(self, x, y):
        if self.current_map is None:
            return (0, 0)
        res = self.current_map.info.resolution
        ox = self.current_map.info.origin.position.x
        oy = self.current_map.info.origin.position.y
        return (int(round((x - ox) / res)), int(round((y - oy) / res)))

    def grid_to_world(self, gx, gy):
        if self.current_map is None:
            return (0.0, 0.0)
        res = self.current_map.info.resolution
        ox = self.current_map.info.origin.position.x
        oy = self.current_map.info.origin.position.y
        return (gx * res + ox, gy * res + oy)

    def grid_path_to_msg(self, path_grid):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.param_global_frame
        for cell in path_grid:
            wx, wy = self.grid_to_world(cell[0], cell[1])
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.param_global_frame
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = GlobalPlanner()
        node.run()
    except rospy.ROSInterruptException:
        pass
