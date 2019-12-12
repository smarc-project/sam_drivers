#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState
from sam_msgs.msg import ThrusterRPMs, ThrusterAngles
import math

class SamJointStateConverter(object):

    def timer_callback(self, event):

        state = JointState()
        state.name = ["sam/thruster_joint_1", "sam/thruster_joint_2"]
        duration = (rospy.Time.now() - self.start_time).to_sec()
        state.position = [duration*vel for vel in self.velocities]
        self.joint_state_pub.publish(state)

    def thruster_callback(self, msg):

        state = JointState()
        state.name = ["sam/thruster_joint_1", "sam/thruster_joint_2"]
        self.velocities = [2.*math.pi/60.*float(msg.thruster_1_rpm), 2.*math.pi/60.*float(msg.thruster_2_rpm)]
        state.velocity = self.velocities
        self.joint_state_pub.publish(state)

    def vector_callback(self, msg):

        state = JointState()
        state.name = ["sam/shaft_joint1", "sam/shaft_joint2"]
        state.position = [msg.thruster_vertical_radians, msg.thruster_horizontal_radians]
        self.joint_state_pub.publish(state)

    def __init__(self):

        rospy.init_node("sam_joint_state_converter", anonymous=True)
        self.start_time = rospy.Time.now()
        self.velocities = [0., 0.]

        self.thruster_sub = rospy.Subscriber("~rpm_cmd", ThrusterRPMs, self.thruster_callback)
        self.vector_sub = rospy.Subscriber("~thrust_vector_cmd", ThrusterAngles, self.vector_callback)

        self.joint_state_pub = rospy.Publisher("/sam/command_states", JointState, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        rospy.spin()

if __name__ == "__main__":
    
    sc = SamJointStateConverter()
