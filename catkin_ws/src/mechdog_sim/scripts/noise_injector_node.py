#!/usr/bin/env python
"""
Noise Injector Node for MechDog Simulation
Subscribes to clean sensor data from Gazebo and publishes noisy data
to emulate real-world sensor imperfections (Sim-to-Real gap mitigation)
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import copy


class NoiseInjector:
    def __init__(self):
        rospy.init_node('noise_injector', anonymous=False)
        
        # Load noise parameters
        self.load_parameters()
        
        # Initialize publishers
        self.scan_pub = rospy.Publisher('/mechdog/scan', LaserScan, queue_size=10)
        self.odom_pub = rospy.Publisher('/mechdog/odom', Odometry, queue_size=10)
        
        # Initialize subscribers
        self.scan_sub = rospy.Subscriber('/gazebo/lidar_clean', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/gazebo/odom_clean', Odometry, self.odom_callback)
        
        # State tracking for drift
        self.odom_drift = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.last_odom_time = rospy.Time.now()
        
        rospy.loginfo("Noise Injector Node initialized")
        
    def load_parameters(self):
        """Load noise injection parameters from ROS parameter server"""
        # LIDAR parameters
        self.lidar_enabled = rospy.get_param('~noise_injection/lidar/enabled', True)
        self.lidar_stddev = rospy.get_param('~noise_injection/lidar/gaussian_noise/stddev', 0.01)
        self.lidar_outlier_prob = rospy.get_param('~noise_injection/lidar/outlier_probability', 0.02)
        self.lidar_dropout_prob = rospy.get_param('~noise_injection/lidar/dropout_probability', 0.01)
        
        # Odometry parameters
        self.odom_enabled = rospy.get_param('~noise_injection/odometry/enabled', True)
        self.odom_pos_stddev = rospy.get_param('~noise_injection/odometry/position_noise/x/stddev', 0.005)
        self.odom_theta_stddev = rospy.get_param('~noise_injection/odometry/position_noise/theta/stddev', 0.01)
        self.odom_linear_drift = rospy.get_param('~noise_injection/odometry/drift/linear', 0.001)
        self.odom_angular_drift = rospy.get_param('~noise_injection/odometry/drift/angular', 0.002)
        
        rospy.loginfo("Noise parameters loaded from parameter server")
        
    def scan_callback(self, msg):
        """Process LIDAR scan with noise injection"""
        if not self.lidar_enabled:
            self.scan_pub.publish(msg)
            return
            
        noisy_scan = copy.deepcopy(msg)
        ranges = np.array(msg.ranges)
        
        # Gaussian noise
        noise = np.random.normal(0, self.lidar_stddev, ranges.shape)
        ranges += noise
        
        # Outliers
        outlier_mask = np.random.random(ranges.shape) < self.lidar_outlier_prob
        if np.any(outlier_mask):
            outlier_values = np.random.uniform(0.05, 0.15, np.sum(outlier_mask))
            ranges[outlier_mask] = outlier_values
            
        # Dropouts (set to max range or nan)
        dropout_mask = np.random.random(ranges.shape) < self.lidar_dropout_prob
        if np.any(dropout_mask):
            ranges[dropout_mask] = msg.range_max
            
        # Clamp to valid range
        ranges = np.clip(ranges, msg.range_min, msg.range_max)
        
        noisy_scan.ranges = ranges.tolist()
        self.scan_pub.publish(noisy_scan)
        
    def odom_callback(self, msg):
        """Process odometry with noise and drift injection"""
        if not self.odom_enabled:
            self.odom_pub.publish(msg)
            return
            
        noisy_odom = copy.deepcopy(msg)
        
        # Calculate time delta for drift accumulation
        current_time = rospy.Time.now()
        dt = (current_time - self.last_odom_time).to_sec()
        self.last_odom_time = current_time
        
        # Accumulate drift
        self.odom_drift['x'] += self.odom_linear_drift * dt
        self.odom_drift['y'] += self.odom_linear_drift * dt
        self.odom_drift['theta'] += self.odom_angular_drift * dt
        
        # Add Gaussian noise to position
        noisy_odom.pose.pose.position.x += np.random.normal(0, self.odom_pos_stddev) + self.odom_drift['x']
        noisy_odom.pose.pose.position.y += np.random.normal(0, self.odom_pos_stddev) + self.odom_drift['y']
        
        # Add noise to orientation (quaternion - simplified)
        # In production, should properly handle quaternion noise
        theta_noise = np.random.normal(0, self.odom_theta_stddev) + self.odom_drift['theta']
        # Simple approximation for small angles
        noisy_odom.pose.pose.orientation.z += theta_noise * 0.5
        noisy_odom.pose.pose.orientation.w = np.sqrt(1 - noisy_odom.pose.pose.orientation.z**2)
        
        self.odom_pub.publish(noisy_odom)
        
    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = NoiseInjector()
        node.run()
    except rospy.ROSInterruptException:
        pass
