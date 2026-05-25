#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class HomeBaseNode:
    def __init__(self):
        rospy.init_node('home_base_node', anonymous=False)
        self.load_parameters()

        self.home_pose = None
        self.current_pose = None
        self.return_in_progress = False
        self.recovery_count = 0
        self.navigation_status = "idle"

        self.goal_pub = rospy.Publisher(
            self.param_goal_topic, PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher(
            self.param_status_topic, String, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(
            self.param_marker_topic, Marker, queue_size=1)

        self.odom_sub = rospy.Subscriber(
            self.param_odom_topic, Odometry, self.odom_callback)
        self.safety_sub = rospy.Subscriber(
            self.param_safety_topic, String, self.safety_callback)

        self.init_timer = rospy.Timer(rospy.Duration(0.5), self.init_callback)
        self.marker_timer = rospy.Timer(rospy.Duration(1.0), self.publish_marker_callback)

        rospy.loginfo("Home Base Node initialized")

    def load_parameters(self):
        self.param_enabled = rospy.get_param('~enabled', True)
        self.param_home_x = rospy.get_param('~position/x', 0.0)
        self.param_home_y = rospy.get_param('~position/y', 0.0)
        self.param_tolerance = rospy.get_param('~position/tolerance', 0.2)
        self.param_trigger = rospy.get_param('~on_failure/trigger_after_recoveries', 3)
        self.param_goal_topic = rospy.get_param('~on_failure/topic', '/mechdog/goal')
        self.param_status_topic = rospy.get_param(
            '~on_failure/status_topic', '/mechdog_experiments/home_base_status')
        self.param_marker_topic = rospy.get_param(
            '~visualization/marker_topic', '/mechdog_experiments/home_base_marker')
        self.param_odom_topic = rospy.get_param('~odom_topic', '/mechdog/odom')
        self.param_safety_topic = rospy.get_param(
            '~safety_topic', '/mechdog/safety_status')
        self.param_safe_zone = rospy.get_param('~safe_zone_radius', 1.0)

    def init_callback(self, event):
        if self.home_pose is None:
            self.home_pose = (self.param_home_x, self.param_home_y)
            rospy.loginfo("Home base set at (%.2f, %.2f)", self.param_home_x, self.param_home_y)
            self.publish_status("HOME_SET")
            self.init_timer.shutdown()

    def odom_callback(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def safety_callback(self, msg):
        if not self.param_enabled:
            return
        status = msg.data
        if status == "EMERGENCY_STOP":
            self.recovery_count += 1
            rospy.logwarn(
                "Home Base: emergency stop #%d detected", self.recovery_count)
            if self.recovery_count >= self.param_trigger:
                rospy.logerr(
                    "Home Base: %d emergencies - returning to home!", self.recovery_count)
                self.return_to_home()

    def return_to_home(self):
        if self.home_pose is None:
            rospy.logwarn("Home Base: no home pose set")
            return
        if self.return_in_progress:
            return

        self.return_in_progress = True
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.home_pose[0]
        goal.pose.position.y = self.home_pose[1]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.publish_status("RETURNING_HOME")
        rospy.loginfo("Home Base: published return goal to (%.2f, %.2f)",
                      self.home_pose[0], self.home_pose[1])

    def is_at_home(self):
        if self.current_pose is None or self.home_pose is None:
            return False
        dx = self.current_pose[0] - self.home_pose[0]
        dy = self.current_pose[1] - self.home_pose[1]
        dist = math.sqrt(dx**2 + dy**2)
        return dist < self.param_tolerance

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_marker_callback(self, event):
        if self.home_pose is None:
            return
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "home_base"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.home_pose[0]
        marker.pose.position.y = self.home_pose[1]
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime = rospy.Duration(0)
        self.marker_pub.publish(marker)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.return_in_progress and self.is_at_home():
                rospy.loginfo("Home Base: reached home safely")
                self.publish_status("AT_HOME")
                self.return_in_progress = False
                self.recovery_count = 0
            rate.sleep()


if __name__ == '__main__':
    try:
        node = HomeBaseNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
