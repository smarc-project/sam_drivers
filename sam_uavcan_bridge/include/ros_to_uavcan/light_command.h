#ifndef ROS_TO_UAV_LIGHT_COMMAND_H
#define ROS_TO_UAV_LIGHT_COMMAND_H

#include <uavcan_ros_bridge.h>
#include <sam_msgs/msg/light_command.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::LightCommand> ros_msg, uavcan_equipment_indication_LightsCommand& uav_msg);

}

#endif
