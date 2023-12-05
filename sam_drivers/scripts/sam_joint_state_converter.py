#!/usr/bin/python3

import rclpy
import sys
import math

from sensor_msgs.msg import JointState
from sam_msgs.msg import ThrusterRPMs, ThrusterAngles

class SamJointStateConverter(object):

    def timer_callback(self):

        state = JointState()
        state.name = ["sam/thruster_joint_1", "sam/thruster_joint_2"]
        duration = (self.rosnode.get_clock().now() - self.start_time).nanoseconds * 1e-9 #no .seconds.... bruh...
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

        rclpy.init(args=sys.argv)
        self.rosnode = rclpy.create_node("sam_joint_state_converter")

        self.start_time = self.rosnode.get_clock().now()
        self.velocities = [0., 0.]

        self.thruster_sub = self.rosnode.create_subscription(ThrusterRPMs, "rpm_cmd",  self.thruster_callback, 10)
        self.vector_sub = self.rosnode.create_subscription(ThrusterAngles, "thrust_vector_cmd", self.vector_callback, 10)

        # TODO: why use hardcoded /sam/ instead of namespacing this? The subs above are...
        self.joint_state_pub = self.rosnode.create_publisher(JointState, "/sam/command_states", 10)

        self.timer = self.rosnode.create_timer(0.1, self.timer_callback) # period in seconds, callback

        rclpy.spin(self.rosnode)

if __name__ == "__main__":
    
    sc = SamJointStateConverter()
