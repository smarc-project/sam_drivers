#ifndef ROS_TO_UAV_LIGHT_COMMAND_H
#define ROS_TO_UAV_LIGHT_COMMAND_H

#include <uavcan/equipment/indication/LightsCommand.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/LightCommand.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::LightCommand& ros_msg, uavcan::equipment::indication::LightsCommand& uav_msg);

}

#endif
