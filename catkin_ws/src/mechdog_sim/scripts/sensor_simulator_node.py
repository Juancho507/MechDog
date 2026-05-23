#!/usr/bin/env python
"""
Sensor Simulator Node for MechDog
Manages sensor simulation and provides interfaces matching hardware sensors
Ensures identical topic structure between simulation and real robot
"""

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class SensorSimulator:
    def __init__(self):
        rospy.init_node('sensor_simulator', anonymous=False)
        
        # Load sensor parameters
        self.load_parameters()
        
        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Monitor sensor health
        self.last_scan_time = rospy.Time.now()
        self.last_odom_time = rospy.Time.now()
        self.sensor_timeout = rospy.Duration(1.0)  # 1 second timeout
        
        # Initialize subscribers for monitoring
        self.scan_sub = rospy.Subscriber('/mechdog/scan', LaserScan, self.scan_monitor_callback)
        self.odom_sub = rospy.Subscriber('/mechdog/odom', Odometry, self.odom_monitor_callback)
        
        # Initialize diagnostic timer
        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.diagnostics_callback)
        
        rospy.loginfo("Sensor Simulator Node initialized")
        
    def load_parameters(self):
        """Load sensor configuration from parameter server"""
        # LIDAR configuration
        self.lidar_frame = rospy.get_param('~sensors/lidar/frame_id', 'lidar_link')
        self.lidar_update_rate = rospy.get_param('~sensors/lidar/update_rate', 20.0)
        
        # Odometry configuration
        self.odom_frame = rospy.get_param('~sensors/odometry/frame_id', 'odom')
        self.odom_child_frame = rospy.get_param('~sensors/odometry/child_frame_id', 'base_footprint')
        self.odom_update_rate = rospy.get_param('~sensors/odometry/update_rate', 50.0)
        
        # TF configuration
        self.tf_publish_rate = rospy.get_param('~tf/publish_rate', 50.0)
        
        rospy.loginfo("Sensor parameters loaded")
        
    def scan_monitor_callback(self, msg):
        """Monitor LIDAR scan messages"""
        self.last_scan_time = rospy.Time.now()
        
        # Validate scan data
        if len(msg.ranges) == 0:
            rospy.logwarn_throttle(5.0, "Received empty LIDAR scan")
            return
            
        # Check for invalid readings
        valid_count = sum(1 for r in msg.ranges if msg.range_min <= r <= msg.range_max)
        valid_ratio = float(valid_count) / len(msg.ranges)
        
        if valid_ratio < 0.5:
            rospy.logwarn_throttle(5.0, 
                f"LIDAR scan quality low: {valid_ratio*100:.1f}% valid readings")
                
    def odom_monitor_callback(self, msg):
        """Monitor odometry messages"""
        self.last_odom_time = rospy.Time.now()
        
        # Validate odometry data
        pos = msg.pose.pose.position
        if abs(pos.x) > 1000 or abs(pos.y) > 1000:
            rospy.logwarn_throttle(5.0, 
                f"Odometry position seems unrealistic: x={pos.x:.2f}, y={pos.y:.2f}")
                
    def diagnostics_callback(self, event):
        """Periodic diagnostics check"""
        current_time = rospy.Time.now()
        
        # Check LIDAR health
        scan_dt = current_time - self.last_scan_time
        if scan_dt > self.sensor_timeout:
            rospy.logwarn_throttle(5.0, 
                f"LIDAR timeout: no data for {scan_dt.to_sec():.1f}s")
                
        # Check odometry health
        odom_dt = current_time - self.last_odom_time
        if odom_dt > self.sensor_timeout:
            rospy.logwarn_throttle(5.0, 
                f"Odometry timeout: no data for {odom_dt.to_sec():.1f}s")
                
        # Log diagnostics summary
        rospy.logdebug(
            f"Sensor status - LIDAR: {scan_dt.to_sec():.2f}s ago, "
            f"Odom: {odom_dt.to_sec():.2f}s ago"
        )
        
    def publish_static_transforms(self):
        """Publish static transforms (should be done by robot_state_publisher normally)"""
        # This is a backup method; normally robot_state_publisher handles this
        pass
        
    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz for monitoring
        
        while not rospy.is_shutdown():
            # Additional processing can be added here
            rate.sleep()


if __name__ == '__main__':
    try:
        node = SensorSimulator()
        node.run()
    except rospy.ROSInterruptException:
        pass
