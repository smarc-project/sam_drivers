#ifndef UAV_TO_ROS_DUAL_THRUSTER_FEEDBACK_H
#define UAV_TO_ROS_DUAL_THRUSTER_FEEDBACK_H

#include <uavcan_ros_bridge.h>
#include <smarc_msgs/msg/dual_thruster_feedback.hpp>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_DualThrusterFeedback& uav_msg, std::shared_ptr<smarc_msgs::msg::DualThrusterFeedback> ros_msg);

}

#endif
