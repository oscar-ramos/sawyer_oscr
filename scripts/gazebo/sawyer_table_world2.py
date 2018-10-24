#!/usr/bin/env python

import sys

import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point

import intera_interface


def load_gazebo_models(
        table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
        table_reference_frame="world",
        block1_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
        block1_reference_frame="world",
        block2_pose=Pose(position=Point(x=0.4225, y=0.0565, z=0.7725)),
        block2_reference_frame="world",
        block3_pose=Pose(position=Point(x=0.4225, y=-0.0135, z=0.7725)),
        block3_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('sawyer_sim_examples')+"/models/"
    block_path = rospkg.RosPack().get_path('sawyer_oscr')+"/urdf/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block1_xml = ''
    block2_xml = ''
    block3_xml = ''
    with open (block_path + "blocks/block1.urdf", "r") as block_file:
        block1_xml=block_file.read().replace('\n', '')
    with open (block_path + "blocks/block2.urdf", "r") as block_file:
        block2_xml=block_file.read().replace('\n', '')
    with open (block_path + "blocks/block3.urdf", "r") as block_file:
        block3_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp1_urdf = spawn_urdf("block1", block1_xml, "/",
                               block1_pose, block1_reference_frame)
        resp2_urdf = spawn_urdf("block2", block2_xml, "/",
                               block2_pose, block2_reference_frame)
        resp3_urdf = spawn_urdf("block3", block3_xml, "/",
                               block3_pose, block3_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block1")
        resp_delete = delete_model("block2")
        resp_delete = delete_model("block3")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))

def main():
    rospy.init_node("sawyer_table_world")
    # Load Gazebo Models via Spawning Services
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)
    # Move to the initial posture
    limb = intera_interface.Limb('right')
    limb.move_to_neutral()
    # Exit
    while not rospy.is_shutdown():
        pass

    return 0

if __name__ == '__main__':
    sys.exit(main())
