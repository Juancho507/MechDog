#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetModelState

rospy.init_node('test_get_model_state', anonymous=True)
rospy.wait_for_service('/gazebo/get_model_state')
get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

resp = get_state('mechdog', 'world')
print('Success:', resp.success)
print('Pose:', resp.pose.position.x, resp.pose.position.y)
print('Status:', resp.status_message)
