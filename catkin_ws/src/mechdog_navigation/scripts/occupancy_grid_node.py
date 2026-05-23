#!/usr/bin/env python
"""
Occupancy Grid Mapping Node for MechDog Navigation
Real-time map construction from sensor data using Bayesian update
100% portable - no simulation dependencies
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import math
import tf


class OccupancyGridMapper:
    def __init__(self):
        rospy.init_node('occupancy_grid_mapper', anonymous=False)
        
        # Load parameters
        self.load_parameters()
        
        # Initialize map
        self.initialize_map()
        
        # State
        self.current_pose = None
        self.last_update_pose = None
        self.tf_listener = tf.TransformListener()
        
        # Publishers
        self.map_pub = rospy.Publisher(
            self.param_map_output, OccupancyGrid, queue_size=1, latch=True)
        self.metadata_pub = rospy.Publisher(
            self.param_metadata_output, MapMetaData, queue_size=1, latch=True)
        
        # Subscribers
        self.scan_sub = rospy.Subscriber(
            self.param_scan_input, LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            self.param_odom_input, Odometry, self.odom_callback, queue_size=1)
        
        # Update timer
        self.update_timer = rospy.Timer(
            rospy.Duration(1.0 / self.param_update_rate),
            self.publish_map_callback)
        
        rospy.loginfo("Occupancy Grid Mapper initialized: %dx%d @ %.2fm resolution",
                     self.map_width, self.map_height, self.param_resolution)
        
    def load_parameters(self):
        """Load parameters from parameter server"""
        # Map dimensions
        self.param_width = rospy.get_param('~occupancy_grid/map/width', 50.0)
        self.param_height = rospy.get_param('~occupancy_grid/map/height', 50.0)
        self.param_resolution = rospy.get_param('~occupancy_grid/map/resolution', 0.05)
        self.param_origin_x = rospy.get_param('~occupancy_grid/map/origin_x', -25.0)
        self.param_origin_y = rospy.get_param('~occupancy_grid/map/origin_y', -25.0)
        
        # Initial state
        self.param_initial_value = rospy.get_param('~occupancy_grid/initial/occupancy_value', -1)
        
        # Update parameters
        self.param_hit_prob = rospy.get_param('~occupancy_grid/update/hit_probability', 0.7)
        self.param_miss_prob = rospy.get_param('~occupancy_grid/update/miss_probability', 0.4)
        self.param_use_log_odds = rospy.get_param('~occupancy_grid/update/use_log_odds', True)
        self.param_max_log_odds = rospy.get_param('~occupancy_grid/update/max_log_odds', 3.5)
        self.param_min_log_odds = rospy.get_param('~occupancy_grid/update/min_log_odds', -2.0)
        self.param_update_rate = rospy.get_param('~occupancy_grid/update/rate', 10.0)
        
        # Ray tracing
        self.param_max_range = rospy.get_param('~occupancy_grid/ray_trace/max_range', 10.0)
        self.param_min_range = rospy.get_param('~occupancy_grid/ray_trace/min_range', 0.1)
        
        # Odometry integration
        self.param_min_translation = rospy.get_param(
            '~occupancy_grid/odometry/min_translation', 0.05)
        self.param_min_rotation = rospy.get_param('~occupancy_grid/odometry/min_rotation', 0.1)
        
        # Topics
        self.param_scan_input = rospy.get_param('~occupancy_grid/topics/scan_input', '/mechdog/scan')
        self.param_odom_input = rospy.get_param('~occupancy_grid/topics/odom_input', '/mechdog/odom')
        self.param_map_output = rospy.get_param('~occupancy_grid/topics/map_output', '/mechdog/map')
        self.param_metadata_output = rospy.get_param(
            '~occupancy_grid/topics/map_metadata_output', '/mechdog/map_metadata')
        
        # Frames
        self.param_global_frame = rospy.get_param('~occupancy_grid/frames/global_frame', 'map')
        self.param_robot_frame = rospy.get_param('~occupancy_grid/frames/robot_frame', 'base_footprint')
        
        # Convert log-odds thresholds
        if self.param_use_log_odds:
            self.log_odds_hit = math.log(self.param_hit_prob / (1 - self.param_hit_prob))
            self.log_odds_miss = math.log(self.param_miss_prob / (1 - self.param_miss_prob))
        
    def initialize_map(self):
        """Initialize occupancy grid map"""
        self.map_width = int(self.param_width / self.param_resolution)
        self.map_height = int(self.param_height / self.param_resolution)
        
        # Initialize with log-odds if using Bayesian update
        if self.param_use_log_odds:
            self.log_odds_map = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        
        # Initialize occupancy grid
        self.occupancy_grid = np.full(
            (self.map_height, self.map_width), 
            self.param_initial_value, 
            dtype=np.int8)
        
        rospy.loginfo("Map initialized: %d x %d cells", self.map_width, self.map_height)
        
    def odom_callback(self, msg):
        """Receive odometry"""
        new_pose = msg.pose.pose
        
        # Check if pose changed significantly
        if self.should_update_map(new_pose):
            self.current_pose = new_pose
            self.last_update_pose = new_pose
            
    def should_update_map(self, new_pose):
        """Check if robot moved enough to trigger map update"""
        if self.last_update_pose is None:
            return True
            
        # Calculate translation
        dx = new_pose.position.x - self.last_update_pose.position.x
        dy = new_pose.position.y - self.last_update_pose.position.y
        translation = math.sqrt(dx**2 + dy**2)
        
        if translation > self.param_min_translation:
            return True
            
        # Calculate rotation
        yaw_new = self.get_yaw_from_quaternion(new_pose.orientation)
        yaw_old = self.get_yaw_from_quaternion(self.last_update_pose.orientation)
        rotation = abs(yaw_new - yaw_old)
        
        if rotation > self.param_min_rotation:
            return True
            
        return False
        
    def scan_callback(self, msg):
        """Receive laser scan and update map"""
        if self.current_pose is None:
            rospy.logwarn_throttle(5.0, "Waiting for odometry...")
            return
            
        # Update map with scan
        self.integrate_scan(msg, self.current_pose)
        
    def integrate_scan(self, scan, pose):
        """Integrate laser scan into occupancy grid"""
        # Get robot position in map coordinates
        robot_x_world = pose.position.x
        robot_y_world = pose.position.y
        robot_yaw = self.get_yaw_from_quaternion(pose.orientation)
        
        robot_x_map, robot_y_map = self.world_to_map(robot_x_world, robot_y_world)
        
        if not self.is_valid_cell(robot_x_map, robot_y_map):
            return
            
        # Process each ray
        angle = scan.angle_min
        for r in scan.ranges:
            # Check if range is valid
            if r < self.param_min_range or r > min(scan.range_max, self.param_max_range):
                angle += scan.angle_increment
                continue
                
            # Calculate end point of ray
            ray_angle = robot_yaw + angle
            end_x_world = robot_x_world + r * math.cos(ray_angle)
            end_y_world = robot_y_world + r * math.sin(ray_angle)
            
            end_x_map, end_y_map = self.world_to_map(end_x_world, end_y_world)
            
            # Ray trace and update cells
            self.ray_trace(robot_x_map, robot_y_map, end_x_map, end_y_map)
            
            angle += scan.angle_increment
            
    def ray_trace(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing"""
        # Mark cells along ray as free
        cells = self.bresenham_line(x0, y0, x1, y1)
        
        for i, (x, y) in enumerate(cells):
            if not self.is_valid_cell(x, y):
                continue
                
            # Mark cells along ray as free (except last)
            if i < len(cells) - 1:
                self.update_cell(x, y, free=True)
            else:
                # Mark end cell as occupied
                self.update_cell(x, y, free=False)
                
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm"""
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
        
    def update_cell(self, x, y, free):
        """Update occupancy probability of cell using Bayesian update"""
        if not self.is_valid_cell(x, y):
            return
            
        if self.param_use_log_odds:
            # Log-odds update
            if free:
                self.log_odds_map[y, x] += self.log_odds_miss
            else:
                self.log_odds_map[y, x] += self.log_odds_hit
                
            # Clamp
            self.log_odds_map[y, x] = np.clip(
                self.log_odds_map[y, x],
                self.param_min_log_odds,
                self.param_max_log_odds
            )
            
            # Convert to probability for occupancy grid
            prob = 1.0 / (1.0 + math.exp(-self.log_odds_map[y, x]))
            self.occupancy_grid[y, x] = int(prob * 100)
        else:
            # Direct probability update
            if free:
                self.occupancy_grid[y, x] = max(0, self.occupancy_grid[y, x] - 1)
            else:
                self.occupancy_grid[y, x] = min(100, self.occupancy_grid[y, x] + 10)
                
    def is_valid_cell(self, x, y):
        """Check if cell is within map bounds"""
        return 0 <= x < self.map_width and 0 <= y < self.map_height
        
    def world_to_map(self, x_world, y_world):
        """Convert world coordinates to map coordinates"""
        x_map = int((x_world - self.param_origin_x) / self.param_resolution)
        y_map = int((y_world - self.param_origin_y) / self.param_resolution)
        return x_map, y_map
        
    def publish_map_callback(self, event):
        """Publish occupancy grid map"""
        self.publish_map()
        
    def publish_map(self):
        """Publish occupancy grid"""
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.param_global_frame
        
        # Metadata
        msg.info.resolution = self.param_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.param_origin_x
        msg.info.origin.position.y = self.param_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten map (row-major order)
        msg.data = self.occupancy_grid.flatten().tolist()
        
        self.map_pub.publish(msg)
        self.metadata_pub.publish(msg.info)
        
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
        mapper = OccupancyGridMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass
