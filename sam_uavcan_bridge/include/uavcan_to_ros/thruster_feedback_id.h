#ifndef UAV_TO_ROS_THRUSTER_FEEDBACK_ID_H
#define UAV_TO_ROS_THRUSTER_FEEDBACK_ID_H

#include <uavcan_ros_bridge.h>
#include <smarc_msgs/msg/thruster_feedback.hpp>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_ThrusterFeedbackID& uav_msg, std::shared_ptr<smarc_msgs::msg::ThrusterFeedback> ros_msg, unsigned char uid);

}

#endif
