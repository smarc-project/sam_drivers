#ifndef ROS_TO_UAV_PANIC_H
#define ROS_TO_UAV_PANIC_H

#include <uavcan_ros_bridge.h>
#include <std_msgs/msg/string.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<std_msgs::msg::String> ros_msg, uavcan_protocol_Panic& uav_msg);

}

#endif
