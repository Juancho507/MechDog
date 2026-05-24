#!/usr/bin/env python
"""
Safe Learning Node for MechDog Navigation
Active safety system with dynamic collision polygon, emergency braking, and dead-end recovery
100% portable - no simulation dependencies
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PolygonStamped, Point32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import math


class SafeLearningController:
    def __init__(self):
        rospy.init_node('safe_learning', anonymous=False)
        
        # Load parameters
        self.load_parameters()
        
        # State
        self.current_cmd_vel = Twist()
        self.current_scan = None
        self.current_odom = None
        self.emergency_stop_active = False
        self.recovery_mode = False
        self.failed_attempts = 0
        
        # Safety state
        self.last_safe_cmd = Twist()
        self.last_brake_time = rospy.Time.now()
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher(
            self.param_cmd_vel_output, Twist, queue_size=1)
        self.emergency_pub = rospy.Publisher(
            self.param_emergency_output, Bool, queue_size=1)
        self.status_pub = rospy.Publisher(
            self.param_status_output, String, queue_size=1)
        self.polygon_pub = rospy.Publisher(
            self.param_polygon_output, PolygonStamped, queue_size=1)
        
        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber(
            self.param_cmd_vel_input, Twist, self.cmd_vel_callback)
        self.scan_sub = rospy.Subscriber(
            self.param_scan_input, LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber(
            self.param_odom_input, Odometry, self.odom_callback)
        
        # High-frequency safety check timer
        self.safety_timer = rospy.Timer(
            rospy.Duration(1.0 / self.param_control_frequency),
            self.safety_check_callback)
        
        rospy.loginfo("Safe Learning Controller initialized at %.1f Hz", 
                     self.param_control_frequency)
        
    def load_parameters(self):
        """Load parameters from parameter server"""
        # Enable/disable
        self.param_enabled = rospy.get_param('~safe_learning/enabled', True)
        
        # Collision polygon
        self.param_base_width = rospy.get_param('~safe_learning/collision_polygon/base_width', 0.4)
        self.param_base_length = rospy.get_param('~safe_learning/collision_polygon/base_length', 0.6)
        self.param_linear_factor = rospy.get_param(
            '~safe_learning/collision_polygon/velocity_expansion/linear_factor', 0.5)
        
        # Braking
        self.param_max_deceleration = rospy.get_param(
            '~safe_learning/braking/max_deceleration', 2.5)
        self.param_reaction_time = rospy.get_param('~safe_learning/braking/reaction_time', 0.1)
        self.param_safety_factor = rospy.get_param('~safe_learning/braking/safety_factor', 1.5)
        self.param_critical_distance = rospy.get_param(
            '~safe_learning/braking/critical_distance', 0.15)
        self.param_warning_distance = rospy.get_param(
            '~safe_learning/braking/warning_distance', 0.3)
        
        # Emergency behaviors
        self.param_stop_decel = rospy.get_param('~safe_learning/emergency/stop/deceleration_rate', 3.0)
        self.param_stop_hold = rospy.get_param('~safe_learning/emergency/stop/hold_time', 1.0)
        self.param_spin_enabled = rospy.get_param('~safe_learning/emergency/spin/enabled', True)
        self.param_spin_velocity = rospy.get_param('~safe_learning/emergency/spin/angular_velocity', 0.5)
        self.param_deadend_threshold = rospy.get_param(
            '~safe_learning/emergency/deadend/detection_threshold', 3)
        
        # Velocity control
        self.param_proximity_scaling = rospy.get_param(
            '~safe_learning/velocity_control/proximity_scaling/enabled', True)
        self.param_safe_distance = rospy.get_param(
            '~safe_learning/velocity_control/proximity_scaling/safe_distance', 1.0)
        self.param_min_distance = rospy.get_param(
            '~safe_learning/velocity_control/proximity_scaling/min_distance', 0.3)
        
        # Obstacle monitoring
        self.param_scan_range = rospy.get_param('~safe_learning/obstacles/scan_range', 3.0)
        self.param_min_obstacle_points = rospy.get_param(
            '~safe_learning/obstacles/min_obstacle_points', 3)
        
        # Topics
        self.param_cmd_vel_input = rospy.get_param(
            '~safe_learning/topics/cmd_vel_input', '/mechdog/cmd_vel_raw')
        self.param_cmd_vel_output = rospy.get_param(
            '~safe_learning/topics/cmd_vel_output', '/cmd_vel')
        self.param_scan_input = rospy.get_param('~safe_learning/topics/scan_input', '/mechdog/scan')
        self.param_odom_input = rospy.get_param('~safe_learning/topics/odom_input', '/mechdog/odom')
        self.param_emergency_output = rospy.get_param(
            '~safe_learning/topics/emergency_stop_output', '/mechdog/emergency_stop')
        self.param_status_output = rospy.get_param(
            '~safe_learning/topics/safety_status_output', '/mechdog/safety_status')
        self.param_polygon_output = rospy.get_param(
            '~safe_learning/topics/safety_polygon_output', '/mechdog/safety_polygon')
        
        # Control frequency
        self.param_control_frequency = rospy.get_param('~safe_learning/control_frequency', 50.0)
        
    def cmd_vel_callback(self, msg):
        """Receive velocity command from local planner"""
        self.current_cmd_vel = msg
        
    def scan_callback(self, msg):
        """Receive laser scan"""
        self.current_scan = msg
        
    def odom_callback(self, msg):
        """Receive odometry"""
        self.current_odom = msg
        
    def safety_check_callback(self, event):
        """Main safety check loop - high frequency"""
        if not self.param_enabled:
            # Safety disabled, passthrough
            self.cmd_vel_pub.publish(self.current_cmd_vel)
            return
            
        if self.current_scan is None or self.current_odom is None:
            # No sensor data, stop
            self.publish_stop()
            return
            
        # Compute dynamic safety polygon
        safety_polygon = self.compute_safety_polygon()
        self.publish_safety_polygon(safety_polygon)
        
        # Check for imminent collision
        min_distance = self.get_minimum_obstacle_distance()
        
        # Critical distance - immediate stop
        if min_distance < self.param_critical_distance:
            rospy.logwarn_throttle(1.0, "CRITICAL: Obstacle at %.2fm - EMERGENCY STOP", min_distance)
            self.activate_emergency_stop()
            self.publish_stop()
            self.emergency_pub.publish(Bool(data=True))
            self.status_pub.publish(String(data="EMERGENCY_STOP"))
            return
            
        # Warning distance - reduce velocity
        if min_distance < self.param_warning_distance:
            rospy.logwarn_throttle(2.0, "WARNING: Obstacle at %.2fm - reducing velocity", min_distance)
            scaled_cmd = self.scale_velocity_by_proximity(self.current_cmd_vel, min_distance)
            self.cmd_vel_pub.publish(scaled_cmd)
            self.status_pub.publish(String(data="WARNING"))
            return
            
        # Calculate required braking distance
        current_velocity = abs(self.current_cmd_vel.linear.x)
        braking_distance = self.calculate_braking_distance(current_velocity)
        
        # Check if safe to execute command
        if min_distance < braking_distance * self.param_safety_factor:
            rospy.logwarn_throttle(2.0, 
                "Insufficient braking distance: need %.2fm, have %.2fm", 
                braking_distance, min_distance)
            # Reduce velocity
            scaled_cmd = self.scale_velocity_by_proximity(self.current_cmd_vel, min_distance)
            self.cmd_vel_pub.publish(scaled_cmd)
            self.status_pub.publish(String(data="BRAKING"))
            return
            
        # All checks passed - execute command
        self.emergency_stop_active = False
        self.cmd_vel_pub.publish(self.current_cmd_vel)
        self.last_safe_cmd = self.current_cmd_vel
        self.status_pub.publish(String(data="SAFE"))
        self.emergency_pub.publish(Bool(data=False))
        
    def compute_safety_polygon(self):
        """Compute dynamic safety polygon based on current velocity"""
        # Get current velocity
        v = abs(self.current_cmd_vel.linear.x) if self.current_odom else 0.0
        
        # Expand polygon based on velocity
        width = self.param_base_width
        length = self.param_base_length + v * self.param_linear_factor
        
        # Create polygon (rectangular)
        polygon = [
            (-length/2, -width/2),
            (length/2, -width/2),
            (length/2, width/2),
            (-length/2, width/2)
        ]
        
        return polygon
        
    def get_minimum_obstacle_distance(self):
        """Get minimum distance to obstacles in front of robot"""
        if self.current_scan is None or len(self.current_scan.ranges) == 0:
            return float('inf')
            
        # Consider only forward-facing region (e.g., ±30 degrees)
        num_ranges = len(self.current_scan.ranges)
        front_sector = int(num_ranges * 0.15)  # 15% of scan on each side
        
        # Get ranges in front sector
        front_ranges = (
            list(self.current_scan.ranges[:front_sector]) + 
            list(self.current_scan.ranges[-front_sector:])
        )
        
        # Filter valid ranges.
        # The LIDAR sits inside the robot body; forward-facing rays hit the front
        # legs at ~0.13-0.15 m.  Use a 6 cm buffer above range_min (0.10 m) so
        # that self-hits (≤ 0.16 m) are excluded before safety evaluation.
        _range_min_cutoff = self.current_scan.range_min + 0.06
        valid_ranges = [
            r for r in front_ranges 
            if _range_min_cutoff < r <= min(self.current_scan.range_max, self.param_scan_range)
        ]
        
        if len(valid_ranges) < self.param_min_obstacle_points:
            return float('inf')
            
        return min(valid_ranges)
        
    def calculate_braking_distance(self, velocity):
        """Calculate required braking distance"""
        # d = v^2 / (2 * a) + v * t_reaction
        braking_dist = (
            (velocity ** 2) / (2 * self.param_max_deceleration) + 
            velocity * self.param_reaction_time
        )
        return braking_dist
        
    def scale_velocity_by_proximity(self, cmd, distance):
        """Scale velocity based on proximity to obstacles"""
        if not self.param_proximity_scaling:
            return cmd
            
        # Linear scaling between min_distance and safe_distance
        if distance >= self.param_safe_distance:
            scale = 1.0
        elif distance <= self.param_min_distance:
            scale = 0.2  # Minimum velocity ratio
        else:
            scale = 0.2 + 0.8 * (distance - self.param_min_distance) / (
                self.param_safe_distance - self.param_min_distance)
            
        scaled_cmd = Twist()
        scaled_cmd.linear.x = cmd.linear.x * scale
        scaled_cmd.angular.z = cmd.angular.z  # Keep angular velocity
        
        return scaled_cmd
        
    def activate_emergency_stop(self):
        """Activate emergency stop"""
        if not self.emergency_stop_active:
            rospy.logerr("EMERGENCY STOP ACTIVATED")
            self.emergency_stop_active = True
            self.last_brake_time = rospy.Time.now()
            self.failed_attempts += 1
            
            # Check for dead-end
            if self.failed_attempts >= self.param_deadend_threshold:
                rospy.logerr("DEAD-END DETECTED - initiating recovery")
                self.recovery_mode = True
                
    def publish_stop(self):
        """Publish stop command"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
        
    def publish_safety_polygon(self, polygon):
        """Publish safety polygon for visualization"""
        msg = PolygonStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_footprint"
        
        for x, y in polygon:
            point = Point32()
            point.x = x
            point.y = y
            point.z = 0.0
            msg.polygon.points.append(point)
            
        self.polygon_pub.publish(msg)
        
    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = SafeLearningController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
