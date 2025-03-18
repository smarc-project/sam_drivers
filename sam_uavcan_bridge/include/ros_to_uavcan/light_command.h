#ifndef ROS_TO_UAV_LIGHT_COMMAND_H
#define ROS_TO_UAV_LIGHT_COMMAND_H

#include <uavcan_ros_bridge.h>
#include <uavcan_ros_msgs/msg/light_command.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<uavcan_ros_msgs::msg::LightCommand> ros_msg, uavcan_equipment_indication_LightsCommand& uav_msg, unsigned char, DefaultTag);

}

#endif
