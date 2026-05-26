#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose

rospy.init_node('spawn_box', anonymous=True)
rospy.wait_for_service('/gazebo/spawn_sdf_model')
spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

with open('/tmp/test_box.sdf') as f:
    model_xml = f.read()

pose = Pose()
pose.position.x = 2.0
pose.position.y = 0.0
pose.position.z = 0.2
pose.orientation.w = 1.0

resp = spawn(
    model_name='test_box',
    model_xml=model_xml,
    robot_namespace='',
    initial_pose=pose,
    reference_frame='world'
)
print(resp.status_message)
