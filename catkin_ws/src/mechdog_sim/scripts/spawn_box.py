#!/usr/bin/env python3
import rospy
import rosservice
from gazebo_msgs.srv import SpawnModel
import os

rospy.init_node('spawn_box', anonymous=True)
rospy.wait_for_service('/gazebo/spawn_sdf_model')
spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

sdf_path = '/tmp/test_box.sdf'
with open(sdf_path, 'r') as f:
    model_xml = f.read()

req = SpawnModel()
req.model_name = 'test_box'
req.model_xml = model_xml
req.robot_namespace = ''
req.initial_pose.position.x = 2.0
req.initial_pose.position.y = 0.0
req.initial_pose.position.z = 0.2
req.initial_pose.orientation.w = 1.0
req.reference_frame = 'world'

resp = spawn(req.model_name, req.model_xml, req.robot_namespace, req.initial_pose, req.reference_frame)
print(resp.status_message)
