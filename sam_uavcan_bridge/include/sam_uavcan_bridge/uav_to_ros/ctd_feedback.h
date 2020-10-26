#ifndef UAV_TO_ROS_CTD_FEEDBACK_H
#define UAV_TO_ROS_CTD_FEEDBACK_H

#include <smarc_uavcan_messages/CTDFeedback.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <smarc_msgs/CTDFeedback.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::CTDFeedback& uav_msg, smarc_msgs::CTDFeedback& ros_msg);

}

#endif
