#ifndef UAV_TO_ROS_PANIC_H
#define UAV_TO_ROS_PANIC_H

#include <uavcan_ros_bridge.h>
#include <std_msgs/msg/string.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_protocol_Panic& uav_msg, std::shared_ptr<std_msgs::msg::String> ros_msg);

}

#endif
