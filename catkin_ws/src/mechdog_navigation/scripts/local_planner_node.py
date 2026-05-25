#!/usr/bin/env python
"""
Local Planner Node for MechDog Navigation
Implements Dynamic Window Approach (DWA) for local trajectory generation
100% portable - no simulation dependencies
"""

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math


class DWALocalPlanner:
    def __init__(self):
        rospy.init_node('local_planner', anonymous=False)
        
        # Load parameters
        self.load_parameters()
        
        # State
        self.global_path = None
        self.current_odom = None
        self.current_scan = None
        self.current_velocity = Twist()
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher(
            self.param_cmd_vel_output, Twist, queue_size=1)
        self.local_plan_pub = rospy.Publisher(
            self.param_local_plan_output, Path, queue_size=1)
        
        # Subscribers
        self.global_path_sub = rospy.Subscriber(
            self.param_global_plan_input, Path, self.global_path_callback)
        self.odom_sub = rospy.Subscriber(
            self.param_odom_input, Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber(
            self.param_scan_input, LaserScan, self.scan_callback)
        
        # Control loop timer
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.param_control_frequency),
            self.control_callback)
        
        rospy.loginfo("Local Planner (DWA) initialized at %.1f Hz", 
                     self.param_control_frequency)
        
    def load_parameters(self):
        """Load parameters from parameter server"""
        # Velocity limits
        self.param_max_vel_x = rospy.get_param('~local_planner/velocity_limits/max_vel_x', 1.0)
        self.param_min_vel_x = rospy.get_param('~local_planner/velocity_limits/min_vel_x', -0.2)
        self.param_max_vel_theta = rospy.get_param(
            '~local_planner/velocity_limits/max_vel_theta', 1.5)
        
        # Acceleration limits
        self.param_acc_lim_x = rospy.get_param('~local_planner/acceleration_limits/acc_lim_x', 2.0)
        self.param_acc_lim_theta = rospy.get_param(
            '~local_planner/acceleration_limits/acc_lim_theta', 3.0)
        
        # DWA parameters
        self.param_vx_samples = rospy.get_param('~local_planner/dwa/vx_samples', 20)
        self.param_vth_samples = rospy.get_param('~local_planner/dwa/vth_samples', 40)
        self.param_sim_time = rospy.get_param('~local_planner/dwa/sim_time', 2.0)
        self.param_sim_granularity = rospy.get_param('~local_planner/dwa/sim_granularity', 0.05)
        
        # Scoring weights
        self.param_path_distance_bias = rospy.get_param(
            '~local_planner/dwa/scoring/path_distance_bias', 32.0)
        self.param_goal_distance_bias = rospy.get_param(
            '~local_planner/dwa/scoring/goal_distance_bias', 20.0)
        self.param_occdist_scale = rospy.get_param(
            '~local_planner/dwa/scoring/occdist_scale', 0.02)
        self.param_speed_bonus = rospy.get_param(
            '~local_planner/dwa/scoring/speed_bonus', 10.0)
        
        # Obstacle avoidance
        self.param_min_obstacle_dist = rospy.get_param(
            '~local_planner/obstacles/min_obstacle_distance', 0.3)
        
        # Goal tolerance
        self.param_xy_goal_tolerance = rospy.get_param(
            '~local_planner/goal_tolerance/xy_goal_tolerance', 0.1)
        self.param_yaw_goal_tolerance = rospy.get_param(
            '~local_planner/goal_tolerance/yaw_goal_tolerance', 0.1)
        
        # Topics
        self.param_global_plan_input = rospy.get_param(
            '~local_planner/topics/global_plan_input', '/mechdog/global_plan')
        self.param_cmd_vel_output = rospy.get_param(
            '~local_planner/topics/cmd_vel_output', '/cmd_vel')
        self.param_odom_input = rospy.get_param(
            '~local_planner/topics/odom_input', '/mechdog/odom')
        self.param_scan_input = rospy.get_param(
            '~local_planner/topics/scan_input', '/mechdog/scan')
        self.param_local_plan_output = rospy.get_param(
            '~local_planner/topics/local_plan_output', '/mechdog/local_plan')
        
        # Control frequency
        self.param_control_frequency = rospy.get_param(
            '~local_planner/control_frequency', 20.0)
        
    def global_path_callback(self, msg):
        """Receive global path"""
        self.global_path = msg
        rospy.logdebug("Received global path with %d waypoints", len(msg.poses))
        
    def odom_callback(self, msg):
        """Receive odometry"""
        self.current_odom = msg
        self.current_velocity.linear.x = msg.twist.twist.linear.x
        self.current_velocity.angular.z = msg.twist.twist.angular.z
        
    def scan_callback(self, msg):
        """Receive laser scan"""
        self.current_scan = msg
        
    def control_callback(self, event):
        """Main control loop"""
        if self.global_path is None or self.current_odom is None or self.current_scan is None:
            return
            
        if len(self.global_path.poses) == 0:
            # No path, stop
            self.publish_velocity(0.0, 0.0)
            return
            
        # Check if goal reached
        if self.is_goal_reached():
            rospy.loginfo("Goal reached!")
            self.publish_velocity(0.0, 0.0)
            return
            
        # Compute DWA velocity command
        best_vel = self.compute_dwa_velocity()
        
        if best_vel is not None:
            self.publish_velocity(best_vel[0], best_vel[1])
        else:
            rospy.logwarn("No valid trajectory found, stopping")
            self.publish_velocity(0.0, 0.0)
            
    def compute_dwa_velocity(self):
        """Compute best velocity using Dynamic Window Approach"""
        # Get dynamic window
        dw = self.get_dynamic_window()
        
        # Sample velocities
        best_score = -float('inf')
        best_vel = None
        best_traj = None
        
        for v in np.linspace(dw[0], dw[1], self.param_vx_samples):
            for w in np.linspace(dw[2], dw[3], self.param_vth_samples):
                # Simulate trajectory
                trajectory = self.simulate_trajectory(v, w)
                
                # Check collision
                if self.check_collision(trajectory):
                    continue
                    
                # Score trajectory
                score = self.score_trajectory(trajectory, v, w)
                
                if score > best_score:
                    best_score = score
                    best_vel = (v, w)
                    best_traj = trajectory
                    
        # Publish best trajectory for visualization
        if best_traj is not None:
            self.publish_local_plan(best_traj)
            
        return best_vel
        
    def get_dynamic_window(self):
        """Compute dynamic window based on current velocity and acceleration limits"""
        dt = 1.0 / self.param_control_frequency
        
        # Current velocities
        v_curr = self.current_velocity.linear.x
        w_curr = self.current_velocity.angular.z
        
        # Dynamic window based on acceleration
        v_min = max(self.param_min_vel_x, v_curr - self.param_acc_lim_x * dt)
        v_max = min(self.param_max_vel_x, v_curr + self.param_acc_lim_x * dt)
        w_min = max(-self.param_max_vel_theta, w_curr - self.param_acc_lim_theta * dt)
        w_max = min(self.param_max_vel_theta, w_curr + self.param_acc_lim_theta * dt)
        
        return [v_min, v_max, w_min, w_max]
        
    def simulate_trajectory(self, v, w):
        """Simulate trajectory for given velocities"""
        trajectory = []
        
        # Get current pose
        x = self.current_odom.pose.pose.position.x
        y = self.current_odom.pose.pose.position.y
        theta = self.get_yaw_from_quaternion(self.current_odom.pose.pose.orientation)
        
        # Simulate forward in time
        num_steps = int(self.param_sim_time / self.param_sim_granularity)
        for i in range(num_steps):
            # Update pose
            x += v * math.cos(theta) * self.param_sim_granularity
            y += v * math.sin(theta) * self.param_sim_granularity
            theta += w * self.param_sim_granularity
            
            trajectory.append((x, y, theta))
            
        return trajectory
        
    def check_collision(self, trajectory):
        """Check if trajectory collides with obstacles"""
        if self.current_scan is None:
            return False
            
        for x, y, theta in trajectory:
            # Check distance to obstacles
            min_dist = self.get_min_obstacle_distance(x, y)
            if min_dist < self.param_min_obstacle_dist:
                return True
                
        return False
        
    def get_min_obstacle_distance(self, x, y):
        """Get minimum distance to obstacles from given position"""
        # Simplified: use current scan relative to current position
        # In production, transform scan to given position
        
        if self.current_scan is None or len(self.current_scan.ranges) == 0:
            return float('inf')
            
        valid_ranges = [r for r in self.current_scan.ranges 
                       if self.current_scan.range_min <= r <= self.current_scan.range_max]
        
        if len(valid_ranges) == 0:
            return float('inf')
            
        return min(valid_ranges)
        
    def score_trajectory(self, trajectory, v, w):
        """Score trajectory based on distance to path, goal, and obstacles"""
        if len(trajectory) == 0:
            return -float('inf')
            
        # Distance to global path
        path_dist = self.distance_to_path(trajectory)
        
        # Distance to goal
        goal_dist = self.distance_to_goal(trajectory[-1])
        
        # Distance to obstacles
        obstacle_dist = self.get_min_obstacle_distance(trajectory[-1][0], trajectory[-1][1])
        
        # Compute score with forward velocity bonus [δ·vx]
        score = (
            self.param_path_distance_bias * (1.0 / (path_dist + 0.1)) +
            self.param_goal_distance_bias * (1.0 / (goal_dist + 0.1)) +
            self.param_occdist_scale * obstacle_dist +
            self.param_speed_bonus * v
        )
        
        return score
        
    def distance_to_path(self, trajectory):
        """Compute minimum distance from trajectory to global path"""
        if self.global_path is None or len(self.global_path.poses) == 0:
            return float('inf')
            
        min_dist = float('inf')
        for traj_point in trajectory:
            for path_pose in self.global_path.poses:
                dx = traj_point[0] - path_pose.pose.position.x
                dy = traj_point[1] - path_pose.pose.position.y
                dist = math.sqrt(dx**2 + dy**2)
                min_dist = min(min_dist, dist)
                
        return min_dist
        
    def distance_to_goal(self, point):
        """Compute distance from point to goal"""
        if self.global_path is None or len(self.global_path.poses) == 0:
            return float('inf')
            
        goal_pose = self.global_path.poses[-1]
        dx = point[0] - goal_pose.pose.position.x
        dy = point[1] - goal_pose.pose.position.y
        return math.sqrt(dx**2 + dy**2)
        
    def is_goal_reached(self):
        """Check if robot has reached goal"""
        if self.global_path is None or len(self.global_path.poses) == 0:
            return False
            
        goal_pose = self.global_path.poses[-1]
        dx = self.current_odom.pose.pose.position.x - goal_pose.pose.position.x
        dy = self.current_odom.pose.pose.position.y - goal_pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        
        return dist < self.param_xy_goal_tolerance
        
    def publish_velocity(self, v, w):
        """Publish velocity command"""
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_vel_pub.publish(cmd)
        
    def publish_local_plan(self, trajectory):
        """Publish local trajectory for visualization"""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        
        for x, y, theta in trajectory:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            
        self.local_plan_pub.publish(path)
        
    def get_yaw_from_quaternion(self, q):
        """Extract yaw from quaternion"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        planner = DWALocalPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
