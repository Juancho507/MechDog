#!/usr/bin/env python
"""
Noise Injector Node for MechDog Simulation
- Subscribes to clean Gazebo sensor data (Range + Odometry)
- Publishes noisy /mechdog/ultrasonic (Range)
- Converts Range → LaserScan for /mechdog/scan (navigation stack compat)
"""
import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
import copy


class NoiseInjector:
    def __init__(self):
        rospy.init_node('noise_injector', anonymous=False)
        self.load_parameters()

        # Publishers
        self.range_pub = rospy.Publisher('/mechdog/ultrasonic', Range, queue_size=10)
        self.scan_pub = rospy.Publisher('/mechdog/scan', LaserScan, queue_size=10)
        self.odom_pub = rospy.Publisher('/mechdog/odom', Odometry, queue_size=10)

        # Subscribers
        self.range_sub = rospy.Subscriber(
            '/gazebo/ultrasonic_clean', Range, self.range_callback)
        self.odom_sub = rospy.Subscriber(
            '/gazebo/odom_clean', Odometry, self.odom_callback)

        # Drift state
        self.odom_drift = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.last_odom_time = rospy.Time.now()

        rospy.loginfo("Noise Injector initialized (ultrasonic + odometry)")

    def load_parameters(self):
        self.ultrasonic_enabled = rospy.get_param(
            '~noise_injection/ultrasonic/enabled', True)
        self.ultrasonic_stddev = rospy.get_param(
            '~noise_injection/ultrasonic/gaussian_noise/stddev', 0.01)
        self.ultrasonic_outlier_prob = rospy.get_param(
            '~noise_injection/ultrasonic/outlier_probability', 0.01)
        self.ultrasonic_dropout_prob = rospy.get_param(
            '~noise_injection/ultrasonic/dropout_probability', 0.02)

        self.odom_enabled = rospy.get_param(
            '~noise_injection/odometry/enabled', True)
        self.odom_pos_stddev = rospy.get_param(
            '~noise_injection/odometry/position_noise/x/stddev', 0.005)
        self.odom_theta_stddev = rospy.get_param(
            '~noise_injection/odometry/position_noise/theta/stddev', 0.01)
        self.odom_linear_drift = rospy.get_param(
            '~noise_injection/odometry/drift/linear', 0.001)
        self.odom_angular_drift = rospy.get_param(
            '~noise_injection/odometry/drift/angular', 0.002)

    def range_callback(self, msg):
        """Receive Range from Gazebo → publish noisy Range + converted LaserScan"""
        noisy_range = copy.deepcopy(msg)

        if self.ultrasonic_enabled:
            raw = msg.range

            # Gaussian noise
            raw += np.random.normal(0, self.ultrasonic_stddev)

            # Outlier
            if np.random.random() < self.ultrasonic_outlier_prob:
                raw = np.random.uniform(0.5, msg.max_range)

            # Dropout → max range (no echo detected)
            if np.random.random() < self.ultrasonic_dropout_prob:
                raw = msg.max_range

            raw = np.clip(raw, msg.min_range, msg.max_range)
            noisy_range.range = float(raw)
        else:
            noisy_range.range = msg.range

        self.range_pub.publish(noisy_range)
        self._publish_as_laserscan(noisy_range)

    def _publish_as_laserscan(self, range_msg):
        """Convert a Range message to a 1-ray LaserScan for nav stack compat."""
        scan = LaserScan()
        scan.header = copy.deepcopy(range_msg.header)
        scan.header.frame_id = 'ultrasonic_link'
        scan.angle_min = 0.0
        scan.angle_max = 0.0
        scan.angle_increment = 0.0
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / 20.0
        scan.range_min = range_msg.min_range
        scan.range_max = range_msg.max_range
        scan.ranges = [range_msg.range]
        scan.intensities = []
        self.scan_pub.publish(scan)

    def odom_callback(self, msg):
        """Receive ground-truth odometry → publish noisy odometry"""
        noisy_odom = copy.deepcopy(msg)
        noisy_odom.header.frame_id = 'odom'
        noisy_odom.child_frame_id = 'base_footprint'

        if not self.odom_enabled:
            self.odom_pub.publish(noisy_odom)
            return

        dt = (rospy.Time.now() - self.last_odom_time).to_sec()
        self.last_odom_time = rospy.Time.now()

        self.odom_drift['x'] += self.odom_linear_drift * dt
        self.odom_drift['y'] += self.odom_linear_drift * dt
        self.odom_drift['theta'] += self.odom_angular_drift * dt

        noisy_odom.pose.pose.position.x += (
            np.random.normal(0, self.odom_pos_stddev) + self.odom_drift['x'])
        noisy_odom.pose.pose.position.y += (
            np.random.normal(0, self.odom_pos_stddev) + self.odom_drift['y'])

        theta_noise = np.random.normal(0, self.odom_theta_stddev) + self.odom_drift['theta']
        z_new = noisy_odom.pose.pose.orientation.z + theta_noise * 0.5
        z_new = float(np.clip(z_new, -1.0, 1.0))
        w_new = math.sqrt(max(0.0, 1.0 - z_new ** 2))
        noisy_odom.pose.pose.orientation.z = z_new
        noisy_odom.pose.pose.orientation.w = w_new

        self.odom_pub.publish(noisy_odom)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = NoiseInjector()
        node.run()
    except rospy.ROSInterruptException:
        pass
