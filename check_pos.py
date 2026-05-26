#!/usr/bin/env python3
import rospy, math
from nav_msgs.msg import Odometry

rospy.init_node("check_pos", anonymous=True)
topic = rospy.wait_for_message("/mechdog/odom", Odometry, timeout=3)
print("Robot position: ({:.3f}, {:.3f}, {:.3f})".format(
    topic.pose.pose.position.x, topic.pose.pose.position.y, topic.pose.pose.position.z))
q = topic.pose.pose.orientation
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
print("Robot yaw: {:.3f} deg".format(yaw*180/math.pi))
print("Vel: ({:.3f}, {:.3f})".format(
    topic.twist.twist.linear.x, topic.twist.twist.angular.z))
