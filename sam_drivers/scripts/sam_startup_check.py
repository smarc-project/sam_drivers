#!/usr/bin/python

import rospy
from sam_msgs.msg import PercentStamped
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header

class StartupCheckServer(object):

    def test_pressure(self):

        rospy.loginfo("Checking pressure...")

        try:
            pressure = rospy.wait_for_message("/uavcan_to_ros_bridge_node/sensor_pressure1", FluidPressure, 3.)
        except rospy.ROSException:
            rospy.loginfo("Could not get pressure on %s, aborting...", "/uavcan_pressure2")
            return False

        rospy.loginfo("Got pressure message with value %f, seems ok!", pressure.fluid_pressure)

        return True

    def test_lcg(self, lcg_setpoint):

        rospy.loginfo("Checking lcg...")

        self.lcg_cmd.publish(value=lcg_setpoint, header=self.header) # publish setpoint 50
        rospy.loginfo("Published lcg setpoint at %f, waiting....", lcg_setpoint)
        rospy.sleep(5.) # wait for 5s to reach 50
        # get lcg feedback, wait for 1s

        try:
            lcg_feedback = rospy.wait_for_message("/uavcan_to_ros_bridge_node/lcg_feedback", PercentStamped, 1.)
        except rospy.ROSException:
            rospy.loginfo("Could not get feedback on %s, aborting...", "/uavcan_to_ros_bridge_node/lcg_feedback")
            return False

        if abs(lcg_feedback.value - lcg_setpoint) > 2.:
            rospy.loginfo("Set point was %f and value was %f, aborting test...", lcg_setpoint, lcg_feedback.value)
            return False
        else:
            rospy.loginfo("Set point was %f and value was %f, seems ok!", lcg_setpoint, lcg_feedback.value)
            return True

    def test_vbs(self, vbs_setpoint):

        rospy.loginfo("Checking vbs...")

        self.vbs_cmd.publish(value=vbs_setpoint, header=self.header) # publish setpoint 50

        rospy.loginfo("Published vbs setpoint at %f, waiting....", vbs_setpoint)

        rospy.sleep(20.) # wait for 5s to reach 50
        # get vbs feedback, wait for 1s

        try:
            vbs_feedback = rospy.wait_for_message("/uavcan_to_ros_bridge_node/vbs_feedback", PercentStamped, 1.)
        except rospy.ROSException:
            rospy.loginfo("Could not get feedback on %s, aborting...", "/uavcan_to_ros_bridge_node/vbs_feedback")
            return False

        if abs(vbs_feedback.value - vbs_setpoint) > 2.:
            rospy.loginfo("Set point was %f and value was %f, aborting test...", vbs_setpoint, vbs_feedback.value)
            return False
        else:
            rospy.loginfo("Set point was %f and value was %f, seems ok!", vbs_setpoint, vbs_feedback.value)
            return True

    def __init__(self):

        self.lcg_cmd = rospy.Publisher('/uavcan_lcg_command', PercentStamped, queue_size=10)
        self.vbs_cmd = rospy.Publisher('/uavcan_vbs_command', PercentStamped, queue_size=10)

        rospy.sleep(1.) # sleep for 1s to wait for stuff to start up

        self.header = Header()
        self.header.stamp = rospy.Time.now()

        self.lcg_cmd.publish(value=0., header=self.header) # publish setpoint 0

        rospy.sleep(1.)

        if not self.test_lcg(50.):
            return

        if not self.test_lcg(88.):
            return

        if not self.test_vbs(0.):
            return

        if not self.test_vbs(100.):
            return

        if not self.test_pressure():
            return

        rospy.loginfo("All tests successful! Exiting...")

if __name__ == "__main__":

    rospy.init_node('sam_startup_check', anonymous=True)

    check_server = StartupCheckServer()
