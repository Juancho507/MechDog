#!/usr/bin/env python
"""
Sensor Simulator Node for MechDog
Manages sensor simulation and provides interfaces matching hardware sensors
Monitors ultrasonic range + odometry health.
"""
import rospy
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry


class SensorSimulator:
    def __init__(self):
        rospy.init_node('sensor_simulator', anonymous=False)
        self.load_parameters()

        self.last_ultrasonic_time = rospy.Time.now()
        self.last_scan_time = rospy.Time.now()
        self.last_odom_time = rospy.Time.now()
        self.sensor_timeout = rospy.Duration(1.0)

        self.range_sub = rospy.Subscriber(
            '/mechdog/ultrasonic', Range, self.range_monitor_callback)
        self.scan_sub = rospy.Subscriber(
            '/mechdog/scan', LaserScan, self.scan_monitor_callback)
        self.odom_sub = rospy.Subscriber(
            '/mechdog/odom', Odometry, self.odom_monitor_callback)

        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.diagnostics_callback)
        rospy.loginfo("Sensor Simulator Node initialized (ultrasonic mode)")

    def load_parameters(self):
        self.ultrasonic_frame = rospy.get_param(
            '~sensors/ultrasonic/frame_id', 'ultrasonic_link')
        self.odom_frame = rospy.get_param(
            '~sensors/odometry/frame_id', 'odom')
        self.odom_child_frame = rospy.get_param(
            '~sensors/odometry/child_frame_id', 'base_footprint')

    def range_monitor_callback(self, msg):
        self.last_ultrasonic_time = rospy.Time.now()
        if msg.range < msg.min_range or msg.range > msg.max_range:
            rospy.logwarn_throttle(
                5.0, f"Ultrasonic out of range: {msg.range:.3f}m")

    def scan_monitor_callback(self, msg):
        self.last_scan_time = rospy.Time.now()

    def odom_monitor_callback(self, msg):
        self.last_odom_time = rospy.Time.now()
        pos = msg.pose.pose.position
        if abs(pos.x) > 1000 or abs(pos.y) > 1000:
            rospy.logwarn_throttle(
                5.0, f"Odometry unrealistic: x={pos.x:.2f}, y={pos.y:.2f}")

    def diagnostics_callback(self, event):
        now = rospy.Time.now()
        for name, last in [('Ultrasonic', self.last_ultrasonic_time),
                           ('Scan', self.last_scan_time),
                           ('Odometry', self.last_odom_time)]:
            dt = now - last
            if dt > self.sensor_timeout:
                rospy.logwarn_throttle(
                    5.0, f"{name} timeout: {dt.to_sec():.1f}s")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        node = SensorSimulator()
        node.run()
    except rospy.ROSInterruptException:
        pass
