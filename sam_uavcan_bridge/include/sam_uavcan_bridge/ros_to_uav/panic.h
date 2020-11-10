#ifndef ROS_TO_UAV_PANIC_H
#define ROS_TO_UAV_PANIC_H

#include <uavcan/protocol/Panic.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <std_msgs/String.h>

namespace ros_to_uav {

template <>
bool convert(const std_msgs::String& ros_msg, uavcan::protocol::Panic& uav_msg);

}

#endif
