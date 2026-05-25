#!/usr/bin/env python
"""
Scan Behavior Node — Active Exploration for MechDog
State machine that rotates the robot 360° when the global planner
cannot find a path (lack of map information due to 15° FOV).
Publishes rotation commands to /cmd_vel and replans after scan.
"""
import rospy
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String


class ScanBehavior:
    def __init__(self):
        rospy.init_node('scan_behavior', anonymous=False)
        self.load_parameters()

        # State
        self._state = 'IDLE'
        self._pending_goal = None
        self._yaw_start = None
        self._yaw_current = 0.0
        self._yaw_travelled = 0.0
        self._no_plan_count = 0
        self._timer_scan = None
        self._timer_plan_wait = None

        # Publishers
        self._cmd_pub = rospy.Publisher(self.param_cmd_vel, Twist, queue_size=1)
        self._goal_pub = rospy.Publisher(self.param_goal_topic, PoseStamped, queue_size=1)

        # Subscribers
        rospy.Subscriber(self.param_goal_topic, PoseStamped, self._goal_cb)
        rospy.Subscriber(self.param_plan_topic, Path, self._plan_cb)
        rospy.Subscriber(self.param_odom_topic, Odometry, self._odom_cb)
        rospy.Subscriber(self.param_status_topic, String, self._status_cb)

        rospy.loginfo("Scan Behavior initialized — scanning angle=%.1f° ω=%.2f rad/s",
                      math.degrees(self.param_scan_angle), self.param_angular_speed)

    # ------------------------------------------------------------------ #
    # Parameters
    # ------------------------------------------------------------------ #

    def load_parameters(self):
        self.param_scan_angle = rospy.get_param('~scan_behavior/scan_angle', 2.0 * math.pi)
        self.param_angular_speed = rospy.get_param('~scan_behavior/angular_speed', 0.3)
        self.param_plan_timeout = rospy.get_param('~scan_behavior/plan_timeout', 5.0)
        self.param_max_consecutive_failures = rospy.get_param(
            '~scan_behavior/max_consecutive_failures', 3)

        self.param_cmd_vel = rospy.get_param('~scan_behavior/topics/cmd_vel', '/cmd_vel')
        self.param_goal_topic = rospy.get_param('~scan_behavior/topics/goal', '/mechdog/goal')
        self.param_plan_topic = rospy.get_param(
            '~scan_behavior/topics/global_plan', '/mechdog/global_plan')
        self.param_odom_topic = rospy.get_param('~scan_behavior/topics/odom', '/mechdog/odom')
        self.param_status_topic = rospy.get_param(
            '~scan_behavior/topics/status', '/mechdog/navigation_status')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _goal_cb(self, msg):
        if self._state != 'IDLE':
            return
        self._pending_goal = msg
        self._no_plan_count = 0
        self._set_state('WAITING_PLAN')
        rospy.loginfo("ScanBehavior: goal received, waiting for plan")
        self._timer_plan_wait = rospy.Timer(
            rospy.Duration(self.param_plan_timeout),
            self._plan_timeout_cb, oneshot=True)

    def _plan_cb(self, msg):
        if self._state not in ('WAITING_PLAN', 'REPLANNING'):
            return
        if len(msg.poses) > 0:
            if self._timer_plan_wait:
                self._timer_plan_wait.shutdown()
                self._timer_plan_wait = None
            rospy.loginfo("ScanBehavior: path found (%d waypoints)", len(msg.poses))
            self._set_state('IDLE')
        else:
            pass

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw_current = math.atan2(siny, cosy)

        if self._state == 'SCANNING' and self._yaw_start is not None:
            raw = self._yaw_current - self._yaw_start
            self._yaw_travelled = raw if raw >= 0 else raw + 2.0 * math.pi
            if self._yaw_travelled >= self.param_scan_angle:
                self._finish_scan()

    def _status_cb(self, msg):
        if msg.data == 'goal_reached':
            self._set_state('DONE')
            rospy.loginfo("ScanBehavior: goal reached")
            if self._timer_scan:
                self._timer_scan.shutdown()
                self._timer_scan = None
            if self._timer_plan_wait:
                self._timer_plan_wait.shutdown()
                self._timer_plan_wait = None
            self._publish_zero()

    # ------------------------------------------------------------------ #
    # Timers
    # ------------------------------------------------------------------ #

    def _plan_timeout_cb(self, event):
        if self._state not in ('WAITING_PLAN', 'REPLANNING'):
            return
        self._no_plan_count += 1
        if self._no_plan_count > self.param_max_consecutive_failures:
            rospy.logwarn("ScanBehavior: %d consecutive failures — aborting",
                          self._no_plan_count)
            self._set_state('ABORTED')
            return
        rospy.loginfo("ScanBehavior: no path found (attempt %d/%d) — starting scan",
                      self._no_plan_count, self.param_max_consecutive_failures)
        self._start_scan()

    def _scan_pub_cb(self, event):
        if self._state != 'SCANNING':
            return
        cmd = Twist()
        cmd.angular.z = self.param_angular_speed
        self._cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ #
    # State transitions
    # ------------------------------------------------------------------ #

    def _set_state(self, new_state):
        old = self._state
        self._state = new_state
        if old == 'SCANNING' and new_state != 'SCANNING':
            if self._timer_scan:
                self._timer_scan.shutdown()
                self._timer_scan = None
            self._publish_zero()

    def _start_scan(self):
        self._yaw_start = self._yaw_current
        self._yaw_travelled = 0.0
        self._set_state('SCANNING')
        rospy.loginfo("ScanBehavior: rotating at %.2f rad/s for %.1f°",
                      self.param_angular_speed,
                      math.degrees(self.param_scan_angle))
        self._timer_scan = rospy.Timer(rospy.Duration(0.1), self._scan_pub_cb)

    def _finish_scan(self):
        self._set_state('REPLANNING')
        duration_s = self.param_scan_angle / self.param_angular_speed
        rospy.loginfo("ScanBehavior: scan complete (%.1f s) — replanning", duration_s)
        if self._pending_goal is not None:
            self._goal_pub.publish(self._pending_goal)
        self._timer_plan_wait = rospy.Timer(
            rospy.Duration(self.param_plan_timeout),
            self._plan_timeout_cb, oneshot=True)

    def _publish_zero(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self._cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ #
    # Main
    # ------------------------------------------------------------------ #

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ScanBehavior()
        node.run()
    except rospy.ROSInterruptException:
        pass
