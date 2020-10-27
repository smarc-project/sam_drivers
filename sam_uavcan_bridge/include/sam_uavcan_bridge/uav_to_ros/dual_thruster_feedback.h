#ifndef UAV_TO_ROS_DUAL_THRUSTER_FEEDBACK_H
#define UAV_TO_ROS_DUAL_THRUSTER_FEEDBACK_H

#include <smarc_uavcan_messages/DualThrusterFeedback.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <smarc_msgs/DualThrusterFeedback.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::DualThrusterFeedback& uav_msg, smarc_msgs::DualThrusterFeedback& ros_msg);

}

#endif
