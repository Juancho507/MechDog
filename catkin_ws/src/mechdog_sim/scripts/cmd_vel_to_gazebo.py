#!/usr/bin/env python
"""
cmd_vel_to_gazebo.py — Kinematic Bridge for Gazebo
Subscribes to /cmd_vel and integrates commanded twist into an
absolute world-frame pose, applied via set_model_state at 100 Hz.

This bypasses ODE friction damping that would otherwise prevent
SetModelState with body-frame twist from achieving the commanded
velocity on a high-mass/high-friction model.

The physical Hiwonder robot does not use this node.
"""

import rospy
import math
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState


class CmdVelToGazeboBridge:
    def __init__(self):
        rospy.init_node('cmd_vel_to_gazebo', anonymous=False)

        self.model_name = rospy.get_param('~model_name', 'mechdog')
        self.rate_hz = rospy.get_param('~rate', 100.0)

        self._last_cmd = Twist()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._initialized = False

        rospy.wait_for_service('/gazebo/set_model_state')
        self._set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self._get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self._init_pose_from_gazebo()

        rospy.Subscriber('/cmd_vel', Twist, self._cmd_callback)
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._apply_cmd)

        rospy.loginfo(
            'cmd_vel_to_gazebo bridge active — %.0f Hz, integrating /cmd_vel → %s (world-frame pose)',
            self.rate_hz, self.model_name,
        )

    def _init_pose_from_gazebo(self):
        try:
            resp = self._get_state(self.model_name, 'world')
            self._x = resp.pose.position.x
            self._y = resp.pose.position.y
            q = resp.pose.orientation
            self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._initialized = True
            rospy.loginfo('Initialized pose from Gazebo: (%.3f, %.3f, %.1f deg)',
                         self._x, self._y, self._yaw * 180.0 / math.pi)
        except Exception as e:
            rospy.logwarn('Could not read initial pose from Gazebo: %s — assuming (0,0,0)', e)
            self._initialized = True

    def _cmd_callback(self, msg):
        self._last_cmd = msg

    def _apply_cmd(self, event):
        if not self._initialized:
            return
        dt = 1.0 / self.rate_hz
        vx = self._last_cmd.linear.x
        vy = self._last_cmd.linear.y
        wz = self._last_cmd.angular.z

        self._yaw += wz * dt
        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)
        self._x += (vx * cos_yaw - vy * sin_yaw) * dt
        self._y += (vx * sin_yaw + vy * cos_yaw) * dt

        try:
            target = ModelState()
            target.model_name = self.model_name
            target.pose = Pose(
                Point(self._x, self._y, 0.2),
                Quaternion(0.0, 0.0, math.sin(self._yaw / 2.0), math.cos(self._yaw / 2.0)),
            )
            target.reference_frame = 'world'
            self._set_state(target)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, 'Gazebo service call failed: %s', e)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        bridge = CmdVelToGazeboBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
