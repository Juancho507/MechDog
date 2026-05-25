#!/usr/bin/env python
"""
cmd_vel_to_gazebo.py — Kinematic Bridge for Gazebo
Subscribes to /cmd_vel and applies the commanded twist
directly to the mechdog model via the Gazebo set_model_state service.

This provides the missing actuation layer for simulation-only runs.
The physical Hiwonder robot does not use this node.
"""

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState


class CmdVelToGazeboBridge:
    def __init__(self):
        rospy.init_node('cmd_vel_to_gazebo', anonymous=False)

        self.model_name = rospy.get_param('~model_name', 'mechdog')
        self.rate_hz = rospy.get_param('~rate', 50.0)

        self._last_cmd = Twist()

        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        self._set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self._get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        rospy.Subscriber('/cmd_vel', Twist, self._cmd_callback)

        self._timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._apply_cmd)

        rospy.loginfo(
            'cmd_vel_to_gazebo bridge active — forwarding /cmd_vel → %s',
            self.model_name,
        )

    def _cmd_callback(self, msg):
        self._last_cmd = msg

    def _apply_cmd(self, event):
        try:
            current = self._get_state(self.model_name, 'world')
            target = ModelState()
            target.model_name = self.model_name
            target.pose = current.pose
            target.twist.linear.x = self._last_cmd.linear.x
            target.twist.linear.y = self._last_cmd.linear.y
            target.twist.angular.z = self._last_cmd.angular.z
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

