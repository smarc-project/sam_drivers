#!/usr/bin/python

import rospy
from uavcan_ros_bridge.msg import ESCStatus
from sam_msgs.msg import ThrusterRPMs
import message_filters

class ESCParser(object):

    def esc_cb(self, esc_fb_msg0, esc_fb_msg1):
	synch_fb = ThrusterRPMs()
        snych_fb.thruster_1_rpm = esc_fb_msg0.esc_index
        synch_fb.thruster_2_rpm = esc_fb_msg1.esc_index
        synch_fb.header.stamp = rospy.get_time()
	self.esc_pub.publish(latest_fb)


    def __init__(self):
        self.esc_subs0 = message_filters.Subscriber("/sam/core/esc_status0_fb", ESCStatus)
        self.esc_subs1 = message_filters.Subscriber("/sam/core/esc_status1_fb", ESCStatus)
        self.esc_pub = rospy.Publisher("/sam/core/rpm_fb", ThrusterRPMs, queue_size=10)
        self.ts = message_filters.TimeSynchronizer([self.esc_subs0, self.esc_subs1], 10)
        self.ts.registerCallback(self.esc_cb)

        rospy.spin()



if __name__ == "__main__":

    rospy.init_node('esc_parser', anonymous=True)
    try:
        esc_parser = ESCParser()
    except rospy.ROSInterruptException:
        pass

