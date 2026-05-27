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
        self._smoothed_vx = 0.0
        self._smoothed_wz = 0.0
        self._smooth_alpha = 0.3  # lower = smoother but more lag
        self._best_score = 0.0
        self._best_info = (0, 0, 0, 0, 0, 0, 0, 0, 0)
        self._last_debug = 0.0
        
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
        self.param_acc_lim_x = rospy.get_param('~local_planner/acceleration_limits/acc_lim_x', 25.0)
        self.param_acc_lim_theta = rospy.get_param(
            '~local_planner/acceleration_limits/acc_lim_theta', 25.0)
        
        # DWA parameters
        self.param_vx_samples = rospy.get_param('~local_planner/dwa/vx_samples', 20)
        self.param_vth_samples = rospy.get_param('~local_planner/dwa/vth_samples', 40)
        self.param_sim_time = rospy.get_param('~local_planner/dwa/sim_time', 4.0)
        self.param_sim_granularity = rospy.get_param('~local_planner/dwa/sim_granularity', 0.05)
        
        # Scoring weights
        self.param_path_distance_bias = rospy.get_param(
            '~local_planner/dwa/scoring/path_distance_bias', 50.0)
        self.param_goal_distance_bias = rospy.get_param(
            '~local_planner/dwa/scoring/goal_distance_bias', 60.0)
        self.param_occdist_scale = rospy.get_param(
            '~local_planner/dwa/scoring/occdist_scale', 15.0)
        self.param_speed_bonus = rospy.get_param(
            '~local_planner/dwa/scoring/speed_bonus', 30.0)
        self.param_progress_bonus = rospy.get_param(
            '~local_planner/dwa/scoring/progress_bonus', 60.0)
        
        # Obstacle avoidance
        self.param_min_obstacle_dist = rospy.get_param(
            '~local_planner/obstacles/min_obstacle_distance', 0.3)
        
        # Goal tolerance
        self.param_xy_goal_tolerance = rospy.get_param(
            '~local_planner/goal_tolerance/xy_goal_tolerance', 0.35)
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
        
    def prune_path(self, path):
        """Remove trailing waypoints that coincide with the robot's current
        position so the path starts ahead of the robot.  Otherwise cross-track
        ~0 for any trajectory near the robot, and the DWA prefers staying put."""
        if self.current_odom is None or len(path.poses) < 2:
            return path
        rx = self.current_odom.pose.pose.position.x
        ry = self.current_odom.pose.pose.position.y
        tol_sq = self.param_xy_goal_tolerance * self.param_xy_goal_tolerance
        keep = 0
        for i, pose in enumerate(path.poses):
            dx = pose.pose.position.x - rx
            dy = pose.pose.position.y - ry
            if dx*dx + dy*dy > tol_sq:
                keep = i
                break
        if keep > 0 and keep < len(path.poses) - 1:
            path.poses = path.poses[keep:]
        return path

    def global_path_callback(self, msg):
        """Receive global path"""
        self.global_path = self.prune_path(msg)
        rospy.logdebug("Received global path with %d waypoints", len(self.global_path.poses))
        
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

        # Final approach: when close to the goal, use a simple proportional
        # controller to drive directly toward it. This avoids DWA oscillation
        # near the goal where forward motion briefly increases Euclidean distance.
        goal_pose = self.global_path.poses[-1].pose
        gx = goal_pose.position.x
        gy = goal_pose.position.y
        rx = self.current_odom.pose.pose.position.x
        ry = self.current_odom.pose.pose.position.y
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        yaw = self.get_yaw_from_quaternion(self.current_odom.pose.pose.orientation)
        goal_angle = math.atan2(gy - ry, gx - rx)
        angle_error = goal_angle - yaw
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        if dist_to_goal < 0.8:
            # Pure pursuit — drive toward goal
            v_target = min(0.15, dist_to_goal * 0.5)
            w_target = 0.8 * angle_error  # P-controller for heading
            self.publish_velocity(v_target, w_target)
            if rospy.get_time() - self._last_debug > 3.0:
                self._last_debug = rospy.get_time()
                rospy.loginfo(
                    "FINAL APPROACH v=%.2f w=%.2f dist=%.2f angle_err=%.1f°",
                    v_target, w_target, dist_to_goal, angle_error * 180 / math.pi)
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
        if self.current_odom is not None:
            rospy.loginfo_throttle(2.0,
                "DWA cycle: pos=(%.2f, %.2f) yaw=%.1f | "
                "v_curr=%.2f w_curr=%.2f",
                self.current_odom.pose.pose.position.x,
                self.current_odom.pose.pose.position.y,
                self.get_yaw_from_quaternion(
                    self.current_odom.pose.pose.orientation) * 180 / math.pi,
                self.current_velocity.linear.x,
                self.current_velocity.angular.z)
        self._best_score = -float('inf')
        self._debug_score = -float('inf')
        self._debug_vel = None
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

        if best_vel is not None and rospy.get_time() - self._last_debug > 5.0:
            self._last_debug = rospy.get_time()
            rospy.loginfo(
                "BEST v=%.2f w=%.2f score=%.1f",
                best_vel[0], best_vel[1], best_score)
            
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
        """Check if ANY point along the trajectory collides with obstacles"""
        if self.current_scan is None or len(trajectory) == 0:
            return False
        
        for pt in trajectory:
            x, y, _ = pt
            d = self.get_min_obstacle_distance(x, y)
            if d < self.param_min_obstacle_dist:
                return True
        return False
        
    def get_min_obstacle_distance(self, x, y):
        """Distance from trajectory point (x,y) to nearest obstacle in world frame.
        
        Previously ignored its arguments and always returned the current scan
        minimum — this made ALL trajectories appear equally blocked regardless
        of direction. Now projects the sensor reading into world coordinates
        and measures distance from the trajectory position to the obstacle.
        """
        if self.current_scan is None or len(self.current_scan.ranges) == 0:
            return float('inf')
            
        valid_ranges = [r for r in self.current_scan.ranges 
                       if self.current_scan.range_min <= r <= self.current_scan.range_max]
        
        if len(valid_ranges) == 0:
            return float('inf')
            
        r_min = min(valid_ranges)
        
        # No obstacle within range
        if r_min >= self.current_scan.range_max - 0.01:
            return float('inf')
        
        # Project obstacle into world frame using robot's current pose
        robot_x = self.current_odom.pose.pose.position.x
        robot_y = self.current_odom.pose.pose.position.y
        yaw = self.get_yaw_from_quaternion(self.current_odom.pose.pose.orientation)
        
        # Obstacle is at (r_min, 0) in sensor frame (forward-facing ultrasonic)
        obs_x = robot_x + r_min * math.cos(yaw)
        obs_y = robot_y + r_min * math.sin(yaw)
        
        return math.hypot(x - obs_x, y - obs_y)
        
    def score_trajectory(self, trajectory, v, w):
        """Score trajectory based on path following, goal attraction, obstacles"""
        if len(trajectory) == 0:
            return -float('inf')
        endpoint = trajectory[-1]
        cross_track, _ = self.path_progress(endpoint)
        goal_dist = self.distance_to_goal(endpoint)
        obstacle_dist = self.get_min_obstacle_distance(endpoint[0], endpoint[1])

        # Path score: smooth decay that never reaches zero.
        # Using 1/(dist+1) instead of 1/(dist+eps) to avoid extreme sensitivity
        # near dist=0 that made the DWA prefer rotation. The +1 keeps the
        # function well-behaved at all distances and always provides attraction.
        path_score = self.param_path_distance_bias / (cross_track + 1.0)

        # Goal score: same smooth formulation
        goal_score = self.param_goal_distance_bias / (goal_dist + 1.0)

        # Obstacle penalty: only within braking distance
        braking = max(abs(v), 0.05) * 0.5 + 0.1
        if obstacle_dist < braking:
            obstacle_cost = self.param_occdist_scale * (1.0 - obstacle_dist / braking)
        else:
            obstacle_cost = 0.0

        # Progress bonus along the path (forward direction)
        total_len = self._path_length()
        dx = endpoint[0] - self.current_odom.pose.pose.position.x
        dy = endpoint[1] - self.current_odom.pose.pose.position.y
        forward_progress = math.hypot(dx, dy)
        progress_score = self.param_progress_bonus * min(forward_progress / max(total_len, 1.0), 0.5)

        # Direction bonus: reward moving toward the goal, even if endpoint is not closer.
        # This prevents getting stuck when the robot faces away from the goal —
        # the initial forward motion increases Euclidean distance to goal, but moving
        # in the right direction is still desirable.
        if self.global_path is not None and len(self.global_path.poses) > 0:
            gp = self.global_path.poses[-1].pose.position
            gdx = gp.x - self.current_odom.pose.pose.position.x
            gdy = gp.y - self.current_odom.pose.pose.position.y
            goal_heading = math.atan2(gdy, gdx)
            traj_heading = math.atan2(dy, dx)
            angle_diff = traj_heading - goal_heading
            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2.0 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2.0 * math.pi
            direction_bonus = self.param_speed_bonus * v * math.cos(angle_diff)
        else:
            direction_bonus = 0.0

        score = (path_score + goal_score - obstacle_cost + progress_score +
                 self.param_speed_bonus * v + direction_bonus)
        return score
        
    def path_progress(self, point):
        """Return (cross_track, progress_index) where progress_index is how far
        along the global path the projection of `point` falls, measured in
        cumulative segment indices (e.g. 4.3 = 4.3 segments from start).
        This rewards forward progress along the path, not just proximity to it.
        """
        if self.global_path is None or len(self.global_path.poses) < 2:
            return (float('inf'), 0.0)
        end_x, end_y = point[0], point[1]
        poses = self.global_path.poses
        best_dist = float('inf')
        best_progress = 0.0

        for i in range(len(poses) - 1):
            x1 = poses[i].pose.position.x
            y1 = poses[i].pose.position.y
            x2 = poses[i + 1].pose.position.x
            y2 = poses[i + 1].pose.position.y
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx*dx + dy*dy
            if seg_len_sq < 1e-12:
                continue
            t = ((end_x - x1)*dx + (end_y - y1)*dy) / seg_len_sq
            if t < 0.0:
                cx, cy = x1, y1
                t_clamped = 0.0
            elif t > 1.0:
                cx, cy = x2, y2
                t_clamped = 1.0
            else:
                cx = x1 + t*dx
                cy = y1 + t*dy
                t_clamped = t
            d = math.hypot(end_x - cx, end_y - cy)
            if d < best_dist:
                best_dist = d
                best_progress = i + t_clamped
        return (best_dist, best_progress)

    def _path_length(self):
        if self.global_path is None or len(self.global_path.poses) < 2:
            return 1.0
        total = 0.0
        poses = self.global_path.poses
        for i in range(len(poses) - 1):
            dx = poses[i+1].pose.position.x - poses[i].pose.position.x
            dy = poses[i+1].pose.position.y - poses[i].pose.position.y
            total += math.hypot(dx, dy)
        return total
        
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
        rx = self.current_odom.pose.pose.position.x
        ry = self.current_odom.pose.pose.position.y
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        dx = rx - gx
        dy = ry - gy
        dist = math.sqrt(dx**2 + dy**2)
        reached = dist < self.param_xy_goal_tolerance
        
        return reached
        
    def publish_velocity(self, v, w):
        """Publish velocity command with exponential smoothing to prevent oscillation"""
        self._smoothed_vx = self._smooth_alpha * v + (1.0 - self._smooth_alpha) * self._smoothed_vx
        self._smoothed_wz = self._smooth_alpha * w + (1.0 - self._smooth_alpha) * self._smoothed_wz
        # Clamp to zero when both input and smoothed value are negligible
        # to prevent denormalized floats (10^-89) from reaching the motor controller.
        if abs(self._smoothed_vx) < 1e-12 and abs(v) < 1e-12:
            self._smoothed_vx = 0.0
        if abs(self._smoothed_wz) < 1e-12 and abs(w) < 1e-12:
            self._smoothed_wz = 0.0
        cmd = Twist()
        cmd.linear.x = self._smoothed_vx
        cmd.angular.z = self._smoothed_wz
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
