#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('goal_pub', anonymous=True)
pub = rospy.Publisher('/mechdog/goal', PoseStamped, queue_size=1, latch=True)
rospy.sleep(0.5)
msg = PoseStamped()
msg.header.frame_id = 'map'
msg.header.stamp = rospy.Time.now()
msg.pose.position.x = 4.0
msg.pose.position.y = 0.0
msg.pose.orientation.w = 1.0
pub.publish(msg)
rospy.loginfo('Goal published: (4.0, 0.0)')
