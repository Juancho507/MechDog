#!/usr/bin/env python
"""
cmd_vel_to_gazebo.py — Kinematic Bridge for Gazebo
Subscribes to /cmd_vel and applies the commanded twist
directly to the mechdog model via the Gazebo set_model_state service.

This provides the missing actuation layer for simulation-only runs.
The physical Hiwonder robot does not use this node.
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class CmdVelToGazeboBridge:
    def __init__(self):
        rospy.init_node('cmd_vel_to_gazebo', anonymous=False)

        self.model_name = rospy.get_param('~model_name', 'mechdog')
        self.rate_hz = rospy.get_param('~rate', 50.0)

        self._last_cmd = Twist()

        rospy.wait_for_service('/gazebo/set_model_state')
        self._set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self._wait_for_model()

        rospy.Subscriber('/cmd_vel', Twist, self._cmd_callback)

        self._timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._apply_cmd)

        rospy.loginfo(
            'cmd_vel_to_gazebo bridge active — forwarding /cmd_vel → %s',
            self.model_name,
        )

    def _wait_for_model(self):
        dummy = ModelState()
        dummy.model_name = self.model_name
        dummy.reference_frame = 'world'
        for i in range(30):
            try:
                self._set_state(dummy)
                rospy.loginfo('Model %s found after ~%ds', self.model_name, i)
                return
            except rospy.ServiceException:
                rospy.sleep(1.0)
        rospy.logwarn(
            'Model %s not found after 30s — bridge will retry on each tick',
            self.model_name,
        )

    def _cmd_callback(self, msg):
        self._last_cmd = msg

    def _apply_cmd(self, event):
        try:
            target = ModelState()
            target.model_name = self.model_name
            # Identity pose relative to current = no pose change
            target.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
            target.twist.linear.x = self._last_cmd.linear.x
            target.twist.linear.y = self._last_cmd.linear.y
            target.twist.angular.z = self._last_cmd.angular.z
            # reference_frame = model name means Gazebo converts body twist
            # to world frame automatically using the model's orientation,
            # so the robot always moves in its own forward direction
            target.reference_frame = self.model_name
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

