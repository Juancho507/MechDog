#!/usr/bin/env python
"""
MechDog Hardware Abstraction Layer (HAL)
Bridges the physical Hiwonder-based quadruped with the ROS navigation stack.
Two operating modes:
  - real:  imports Hiwonder libraries to control the physical robot
  - simulation:  prints velocity commands (for testing the HAL logic)

Topic maps:
  INPUT:  /cmd_vel  (geometry_msgs/Twist)  ← from safe_learning node
  OUTPUT: /mechdog/ultrasonic  (sensor_msgs/Range)  ← from I2C sonar
  OUTPUT: /mechdog/scan  (sensor_msgs/LaserScan)  ← Range→LaserScan conversion
"""
import math
import rospy
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
import copy

# ---------------------------------------------------------------------------
# Mock Hiwonder API  (used when Hiwonder libraries are unavailable)
# ---------------------------------------------------------------------------
HAS_HIWONDER = False
try:
    from Hiwonder import Hiwonder_IIC, mechdog
    HAS_HIWONDER = True
except ImportError:
    pass


class HiwonderSonar:
    """Wrapper around the physical I2C sonar sensor."""
    def __init__(self, bus=1, address=0x77):
        self.bus = bus
        self.address = address
        self._driver = None
        if HAS_HIWONDER:
            self._driver = Hiwonder_IIC.I2CSonar(bus, address)

    def read_distance(self):
        """Returns distance in meters, or NaN on failure."""
        if HAS_HIWONDER and self._driver is not None:
            try:
                raw = self._driver.getDistance()
                return raw / 100.0  # convert cm → m
            except Exception:
                return float('nan')
        else:
            # Simulation stub: return 2.5 m (clear path dummy)
            return 2.0


class MechDogChassis:
    """Wrapper around the physical Hiwonder mechdog API."""
    def __init__(self):
        self._chassis = None
        if HAS_HIWONDER:
            self._chassis = mechdog.MechDog()
            try:
                self._chassis.init()
            except Exception as e:
                rospy.logwarn(f"Hiwonder chassis init failed: {e}")

    def set_velocity(self, linear_x, angular_z):
        if self._chassis is not None:
            try:
                self._chassis.set_velocity(linear_x, angular_z)
            except Exception as e:
                rospy.logwarn(f"Velocity command failed: {e}")
        else:
            rospy.loginfo(f"[HAL SIM] cmd_vel → linear={linear_x:.2f}, angular={angular_z:.2f}")


# ---------------------------------------------------------------------------
# ROS Node
# ---------------------------------------------------------------------------

class MechDogHardwareInterface:
    def __init__(self):
        rospy.init_node('mechdog_hardware_interface', anonymous=False)

        mode = rospy.get_param('~mode', 'hardware')
        self.simulate = mode == 'simulation'

        # Hardware drivers
        self.sonar = HiwonderSonar()
        self.chassis = MechDogChassis()

        # Ultrasonic parameters
        self.range_min = rospy.get_param('~range_min', 0.02)
        self.range_max = rospy.get_param('~range_max', 3.0)
        self.fov = rospy.get_param('~fov', 0.26)
        self.update_rate = rospy.get_param('~update_rate', 20.0)

        # Publishers
        self.range_pub = rospy.Publisher('/mechdog/ultrasonic', Range, queue_size=10)
        self.scan_pub = rospy.Publisher('/mechdog/scan', LaserScan, queue_size=10)

        # Subscriber
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Sensor polling timer
        dt = 1.0 / self.update_rate
        self.sensor_timer = rospy.Timer(rospy.Duration(dt), self.sensor_poll_callback)

        rospy.loginfo(f"MechDog HAL initialized (mode={mode})")

    def cmd_vel_callback(self, msg):
        """Translate ROS Twist → Hiwonder API."""
        self.chassis.set_velocity(msg.linear.x, msg.angular.z)

    def sensor_poll_callback(self, event):
        """Read physical sonar → publish Range + LaserScan."""
        dist = self.sonar.read_distance()
        stamp = rospy.Time.now()

        # --- Publish Range ---
        range_msg = Range()
        range_msg.header.stamp = stamp
        range_msg.header.frame_id = 'ultrasonic_link'
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = self.fov
        range_msg.min_range = self.range_min
        range_msg.max_range = self.range_max
        range_msg.range = dist if math.isfinite(dist) else self.range_max
        self.range_pub.publish(range_msg)

        # --- Publish LaserScan (1 ray, for navigation stack compatibility) ---
        scan = LaserScan()
        scan.header = copy.deepcopy(range_msg.header)
        scan.angle_min = -self.fov / 2.0
        scan.angle_max = self.fov / 2.0
        scan.angle_increment = self.fov
        scan.time_increment = 0.0
        scan.scan_time = dt = 1.0 / self.update_rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [dist if math.isfinite(dist) else self.range_max]
        scan.intensities = []
        self.scan_pub.publish(scan)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MechDogHardwareInterface()
        node.run()
    except rospy.ROSInterruptException:
        pass
