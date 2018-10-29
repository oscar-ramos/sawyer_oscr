#!/usr/bin/python
#
# Move the robot to its initial position and open the gripper
#

import rospy
import intera_interface


def main():
    # Initialize the node
    rospy.init_node('GoToInitial')

    # Initialize limb interface
    limb = intera_interface.Limb('right')
    # Move arm to the initial position
    limb.move_to_neutral()
    # Initialize gripper interface
    try:
        gripper = intera_interface.Gripper('right_gripper')
    except ValueError:
        rospy.logerr("Could not detect a gripper")
        return
    # Open the gripper
    gripper.open()


if __name__ == '__main__':
    main()
