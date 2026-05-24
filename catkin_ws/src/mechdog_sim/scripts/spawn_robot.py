#!/usr/bin/env python3
"""
Spawn MechDog in Gazebo — idempotent.
Deletes any existing 'mechdog' model before spawning to prevent
the 'entity already exists' error on re-launch.
"""
import sys
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


def main():
    rospy.init_node('spawn_mechdog', anonymous=False)

    # ── 1. Delete old model if it exists ────────────────────────────────────
    rospy.loginfo("Checking for existing 'mechdog' model...")
    try:
        rospy.wait_for_service('/gazebo/delete_model', timeout=10.0)
        delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete('mechdog')
        if resp.success:
            rospy.loginfo("Deleted existing mechdog model.")
            rospy.sleep(1.0)   # let Gazebo settle
    except Exception:
        pass   # model didn't exist — that's fine

    # ── 2. Read URDF from parameter server ──────────────────────────────────
    if not rospy.has_param('robot_description'):
        rospy.logerr("robot_description parameter not found. Aborting spawn.")
        sys.exit(1)
    urdf_xml = rospy.get_param('robot_description')

    # ── 3. Spawn ─────────────────────────────────────────────────────────────
    rospy.loginfo("Spawning mechdog...")
    rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=60.0)
    spawn = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

    pose = Pose(
        position=Point(
            x=rospy.get_param('/simulation/robot/spawn_x', 0.0),
            y=rospy.get_param('/simulation/robot/spawn_y', 0.0),
            z=rospy.get_param('/simulation/robot/spawn_z', 0.2),
        ),
        orientation=Quaternion(x=0, y=0, z=0, w=1),
    )

    resp = spawn(
        model_name='mechdog',
        model_xml=urdf_xml,
        robot_namespace='',
        initial_pose=pose,
        reference_frame='world',
    )

    if resp.success:
        rospy.loginfo("Spawn successful: %s", resp.status_message)
    else:
        rospy.logerr("Spawn failed: %s", resp.status_message)
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
