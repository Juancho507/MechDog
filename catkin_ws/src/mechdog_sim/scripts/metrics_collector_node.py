#!/usr/bin/env python
"""
Metrics Collector Node for MechDog Simulation
Collects performance metrics during simulation runs for Sim-to-Real comparison
Records: sensor quality, navigation accuracy, safety events, computational timing
"""

import rospy
import csv
import os
from datetime import datetime
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np


class MetricsCollector:
    def __init__(self):
        rospy.init_node('metrics_collector', anonymous=False)
        
        # Metrics storage
        self.metrics = {
            'scan_count': 0,
            'odom_count': 0,
            'emergency_stops': 0,
            'total_distance': 0.0,
            'max_velocity': 0.0,
            'scan_quality': [],
            'path_deviations': [],
            'computation_times': []
        }
        
        # State tracking
        self.last_pose = None
        self.last_time = None
        self.start_time = rospy.Time.now()
        self.global_path = None
        
        # Output directory
        self.output_dir = rospy.get_param('~output_dir', '/tmp/mechdog_metrics')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # Initialize subscribers
        self.scan_sub = rospy.Subscriber('/mechdog/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/mechdog/odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/mechdog/global_plan', Path, self.path_callback)
        self.emergency_sub = rospy.Subscriber('/mechdog/emergency_stop', Bool, self.emergency_callback)
        
        # Initialize periodic save timer
        self.save_timer = rospy.Timer(rospy.Duration(10.0), self.save_metrics_callback)
        
        rospy.loginfo(f"Metrics Collector initialized. Output: {self.output_dir}")
        
    def scan_callback(self, msg):
        """Collect LIDAR scan metrics"""
        self.metrics['scan_count'] += 1
        
        # Calculate scan quality
        ranges = np.array(msg.ranges)
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        valid_ratio = np.sum(valid_mask) / len(ranges)
        
        self.metrics['scan_quality'].append({
            'timestamp': rospy.Time.now().to_sec(),
            'valid_ratio': valid_ratio,
            'mean_range': np.mean(ranges[valid_mask]) if np.any(valid_mask) else 0.0,
            'min_range': np.min(ranges[valid_mask]) if np.any(valid_mask) else msg.range_max
        })
        
    def odom_callback(self, msg):
        """Collect odometry metrics"""
        self.metrics['odom_count'] += 1
        
        current_pose = msg.pose.pose.position
        current_time = msg.header.stamp
        
        # Calculate distance traveled
        if self.last_pose is not None:
            dx = current_pose.x - self.last_pose.x
            dy = current_pose.y - self.last_pose.y
            distance = np.sqrt(dx**2 + dy**2)
            self.metrics['total_distance'] += distance
            
            # Compute velocity from pose deltas (robust: twist is always zero from set_model_state)
            if self.last_time is not None:
                dt = (current_time - self.last_time).to_sec()
                if dt > 0:
                    vel_from_delta = distance / dt
                    if vel_from_delta > self.metrics['max_velocity']:
                        self.metrics['max_velocity'] = vel_from_delta
            
            # Calculate path deviation if global path is available
            if self.global_path is not None:
                deviation = self.calculate_path_deviation(current_pose)
                if deviation is not None:
                    self.metrics['path_deviations'].append({
                        'timestamp': rospy.Time.now().to_sec(),
                        'deviation': deviation
                    })
        
        # Track max velocity
        linear_vel = msg.twist.twist.linear.x
        if abs(linear_vel) > self.metrics['max_velocity']:
            self.metrics['max_velocity'] = abs(linear_vel)
            
        self.last_pose = current_pose
        self.last_time = current_time
        
    def path_callback(self, msg):
        """Store global path for deviation calculation"""
        self.global_path = msg
        rospy.loginfo(f"Received global path with {len(msg.poses)} waypoints")
        
    def emergency_callback(self, msg):
        """Count emergency stop events"""
        if msg.data:
            self.metrics['emergency_stops'] += 1
            rospy.logwarn(f"Emergency stop event #{self.metrics['emergency_stops']} recorded")
            
    def calculate_path_deviation(self, current_pos):
        """Calculate minimum distance from current position to global path"""
        if self.global_path is None or len(self.global_path.poses) == 0:
            return None
            
        min_dist = float('inf')
        for pose_stamped in self.global_path.poses:
            path_pos = pose_stamped.pose.position
            dist = np.sqrt(
                (current_pos.x - path_pos.x)**2 + 
                (current_pos.y - path_pos.y)**2
            )
            if dist < min_dist:
                min_dist = dist
                
        return min_dist
        
    def save_metrics_callback(self, event):
        """Periodically save metrics to file"""
        self.save_metrics()
        
    def save_metrics(self):
        """Save collected metrics to CSV files (overwrites single summary file)"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Overwrite single summary file (avoids 200+ files per session)
        summary_file = os.path.join(self.output_dir, "metrics_summary_latest.csv")
        with open(summary_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['Metric', 'Value'])
            writer.writerow(['Scan Count', self.metrics['scan_count']])
            writer.writerow(['Odom Count', self.metrics['odom_count']])
            writer.writerow(['Emergency Stops', self.metrics['emergency_stops']])
            writer.writerow(['Total Distance (m)', f"{self.metrics['total_distance']:.3f}"])
            writer.writerow(['Max Velocity (m/s)', f"{self.metrics['max_velocity']:.3f}"])
            writer.writerow(['Runtime (s)', f"{(rospy.Time.now() - self.start_time).to_sec():.1f}"])
            
            if self.metrics['path_deviations']:
                deviations = [d['deviation'] for d in self.metrics['path_deviations']]
                writer.writerow(['Mean Path Deviation (m)', f"{np.mean(deviations):.4f}"])
                writer.writerow(['Max Path Deviation (m)', f"{np.max(deviations):.4f}"])
                
        # Save time-series data only on shutdown to reduce file count
        # (timer save only updates summary; full detail saved at the end)
                    
        rospy.loginfo(f"Metrics summary saved (counts: scan={self.metrics['scan_count']}, odom={self.metrics['odom_count']})")
        
    def shutdown_hook(self):
        """Save final metrics on shutdown (with full time-series)"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        rospy.loginfo("Saving final metrics before shutdown...")
        self.save_metrics()
        
        # Save scan quality time series (only on shutdown)
        if self.metrics['scan_quality']:
            scan_file = os.path.join(self.output_dir, f"scan_quality_{timestamp}.csv")
            with open(scan_file, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'Valid Ratio', 'Mean Range', 'Min Range'])
                for entry in self.metrics['scan_quality']:
                    writer.writerow([
                        entry['timestamp'],
                        entry['valid_ratio'],
                        entry['mean_range'],
                        entry['min_range']
                    ])
                    
        # Save path deviations (only on shutdown)
        if self.metrics['path_deviations']:
            dev_file = os.path.join(self.output_dir, f"path_deviations_{timestamp}.csv")
            with open(dev_file, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'Deviation (m)'])
                for entry in self.metrics['path_deviations']:
                    writer.writerow([entry['timestamp'], entry['deviation']])
        
    def run(self):
        """Main loop"""
        rospy.on_shutdown(self.shutdown_hook)
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MetricsCollector()
        node.run()
    except rospy.ROSInterruptException:
        pass
