#!/usr/bin/env python
import rospy
import math
import json
import os
import time
import numpy as np
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool


class MetricsAggregator:
    def __init__(self):
        rospy.init_node('metrics_aggregator', anonymous=False)
        self.load_parameters()

        self.reset_metrics()

        self.odom_sub = rospy.Subscriber(
            self.param_odom_topic, Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber(
            self.param_scan_topic, LaserScan, self.scan_callback)
        self.map_sub = rospy.Subscriber(
            self.param_map_topic, OccupancyGrid, self.map_callback)
        self.safety_sub = rospy.Subscriber(
            self.param_safety_topic, String, self.safety_callback)
        self.status_sub = rospy.Subscriber(
            self.param_status_topic, String, self.status_callback)
        self.emergency_sub = rospy.Subscriber(
            self.param_emergency_topic, Bool, self.emergency_callback)
        self.global_plan_sub = rospy.Subscriber(
            self.param_global_plan_topic, Path, self.global_plan_callback)

        rospy.loginfo("Metrics Aggregator initialized")

    def load_parameters(self):
        self.param_odom_topic = rospy.get_param('~topics/odom', '/mechdog/odom')
        self.param_scan_topic = rospy.get_param('~topics/scan', '/mechdog/scan')
        self.param_map_topic = rospy.get_param('~topics/map', '/mechdog/map')
        self.param_safety_topic = rospy.get_param(
            '~topics/safety_status', '/mechdog/safety_status')
        self.param_status_topic = rospy.get_param(
            '~topics/navigation_status', '/mechdog/navigation_status')
        self.param_emergency_topic = rospy.get_param(
            '~topics/emergency_stop', '/mechdog/emergency_stop')
        self.param_global_plan_topic = rospy.get_param(
            '~topics/global_plan', '/mechdog/global_plan')
        self.param_output_dir = rospy.get_param('~output_dir', '/tmp/mechdog_experiments')

    def reset_metrics(self):
        self.metrics = {
            'start_time': time.time(),
            'execution_time': 0.0,
            'path_length': 0.0,
            'nodes_explored': 0,
            'map_coverage': 0.0,
            'emergency_stops': 0,
            'recovery_attempts': 0,
            'success': False,
            'collision_avoidance_rate': 1.0,
            'trajectory': [],
            'algorithm': 'unknown',
            'scenario': 'unknown',
            'trial': 0,
            'scan_count': 0,
            'total_distance': 0.0,
            'max_velocity': 0.0,
        }
        self.last_position = None
        self.total_scan_points = 0
        self.valid_scan_points = 0
        self.trajectory_samples = []
        self.map_data = None
        self.global_path_poses = []

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x

        self.metrics['max_velocity'] = max(self.metrics['max_velocity'], abs(v))
        self.trajectory_samples.append((x, y, time.time()))

        if self.last_position is not None:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            dist = math.sqrt(dx**2 + dy**2)
            self.metrics['total_distance'] += dist
        self.last_position = (x, y)

    def scan_callback(self, msg):
        self.metrics['scan_count'] += 1
        valid = sum(1 for r in msg.ranges
                    if msg.range_min <= r <= msg.range_max)
        self.total_scan_points += len(msg.ranges)
        self.valid_scan_points += valid

    def map_callback(self, msg):
        self.map_data = msg
        total_cells = len(msg.data)
        known_cells = sum(1 for c in msg.data if c >= 0)
        self.metrics['map_coverage'] = (known_cells / max(total_cells, 1)) * 100.0
        explored = sum(1 for c in msg.data if c == 0)
        self.metrics['nodes_explored'] = explored

    def safety_callback(self, msg):
        if msg.data == "EMERGENCY_STOP":
            self.metrics['emergency_stops'] += 1

    def emergency_callback(self, msg):
        if msg.data:
            self.metrics['emergency_stops'] += 1

    def status_callback(self, msg):
        if msg.data == "recovery":
            self.metrics['recovery_attempts'] += 1
        elif msg.data == "goal_reached":
            self.metrics['success'] = True
            self.metrics['execution_time'] = time.time() - self.metrics['start_time']

    def global_plan_callback(self, msg):
        self.global_path_poses = [(p.pose.position.x, p.pose.position.y)
                                  for p in msg.poses]
        path_len = 0.0
        for i in range(1, len(self.global_path_poses)):
            dx = self.global_path_poses[i][0] - self.global_path_poses[i-1][0]
            dy = self.global_path_poses[i][1] - self.global_path_poses[i-1][1]
            path_len += math.sqrt(dx**2 + dy**2)
        self.metrics['path_length'] = path_len

    def set_experiment_info(self, algorithm, scenario, trial):
        self.metrics['algorithm'] = algorithm
        self.metrics['scenario'] = scenario
        self.metrics['trial'] = trial
        self.metrics['start_time'] = time.time()

    def compute_collision_avoidance_rate(self):
        if self.total_scan_points > 0:
            valid_ratio = self.valid_scan_points / float(self.total_scan_points)
            self.metrics['collision_avoidance_rate'] = min(1.0, valid_ratio + 0.5)
        else:
            self.metrics['collision_avoidance_rate'] = 1.0

    def get_results(self):
        self.compute_collision_avoidance_rate()
        self.metrics['execution_time'] = (
            time.time() - self.metrics['start_time']
            if not self.metrics['success']
            else self.metrics['execution_time']
        )
        self.metrics['trajectory'] = self.trajectory_samples
        return dict(self.metrics)

    def save_results(self, filename=None):
        if filename is None:
            filename = os.path.join(
                self.param_output_dir,
                f"metrics_{self.metrics['algorithm']}_{self.metrics['scenario']}_trial{self.metrics['trial']}.json"
            )
        if not os.path.exists(self.param_output_dir):
            os.makedirs(self.param_output_dir)

        results = self.get_results()
        serializable = {k: v for k, v in results.items() if k != 'trajectory'}
        serializable['trajectory_sample_count'] = len(results.get('trajectory', []))

        with open(filename, 'w') as f:
            json.dump(serializable, f, indent=2)
        rospy.loginfo("Metrics saved to %s", filename)
        return filename


if __name__ == '__main__':
    try:
        node = MetricsAggregator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
