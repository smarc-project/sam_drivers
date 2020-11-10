#ifndef UAV_TO_ROS_PANIC_H
#define UAV_TO_ROS_PANIC_H

#include <uavcan/protocol/Panic.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <std_msgs/String.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::protocol::Panic& uav_msg, std_msgs::String& ros_msg);

}

#endif
