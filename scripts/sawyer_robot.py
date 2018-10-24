
import numpy as np

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class SawyerRobot(object):

    def __init__(self):
        # Topic that contains the sensed joint configuration
        rospy.Subscriber("/robot/joint_states", JointState, self._readJoints)
        # Sensed joint configuration
        self.jstate_ = 7*[0.,]
        # For the Gazebo simulation
        ns = "/robot/right_joint_position_controller/joints"
        topic0 = ns + "/right_j0_controller/command"
        topic1 = ns + "/right_j1_controller/command"
        topic2 = ns + "/right_j2_controller/command"
        topic3 = ns + "/right_j3_controller/command"
        topic4 = ns + "/right_j4_controller/command"
        topic5 = ns + "/right_j5_controller/command"
        topic6 = ns + "/right_j6_controller/command"
        self.pub0 = rospy.Publisher(topic0, Float64, queue_size=10)
        self.pub1 = rospy.Publisher(topic1, Float64, queue_size=10)
        self.pub2 = rospy.Publisher(topic2, Float64, queue_size=10)
        self.pub3 = rospy.Publisher(topic3, Float64, queue_size=10)
        self.pub4 = rospy.Publisher(topic4, Float64, queue_size=10)
        self.pub5 = rospy.Publisher(topic5, Float64, queue_size=10)
        self.pub6 = rospy.Publisher(topic6, Float64, queue_size=10)

    def setJointNames(self, joint_names):
        self.joint_names = joint_names
        
    def publish(self, joints, joints_extra=False):
        """
        Publish the joints to the simulated robot in Gazebo

        """
        self.pub0.publish(joints[0])
        self.pub1.publish(joints[1])
        self.pub2.publish(joints[2])
        self.pub3.publish(joints[3])
        self.pub4.publish(joints[4])
        self.pub5.publish(joints[5])
        self.pub6.publish(joints[6])

    def _readJoints(self, msg):
        """
        Callback to store the sensed joint positions from joint_states topic

        """
        # jstate_ must always be a valid joint configuration
        if (len(msg.position)>2):
            self.jstate_ = msg.position

    def getJointState(self):
        return np.array(self.jstate_[3:])
