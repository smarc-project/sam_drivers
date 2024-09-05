#ifndef UAV_TO_ROS_CTD_FEEDBACK_H
#define UAV_TO_ROS_CTD_FEEDBACK_H

#include <uavcan_ros_bridge.h>
#include <smarc_msgs/msg/ctd.hpp>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_CTDFeedback& uav_msg, std::shared_ptr<smarc_msgs::msg::CTD> ros_msg);

}

#endif
