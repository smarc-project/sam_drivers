#ifndef UAV_TO_ROS_THRUSTER_FEEDBACK_ID_H
#define UAV_TO_ROS_THRUSTER_FEEDBACK_ID_H

#include <smarc_uavcan_messages/ThrusterFeedbackID.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <smarc_msgs/ThrusterFeedback.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<smarc_uavcan_messages::ThrusterFeedbackID>& uav_msg, smarc_msgs::ThrusterFeedback& ros_msg, unsigned char uid);

}

#endif
