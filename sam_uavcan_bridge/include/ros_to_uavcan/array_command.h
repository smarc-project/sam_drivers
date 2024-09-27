#ifndef ROS_TO_UAV_ARRAY_COMMAND_H
#define ROS_TO_UAV_ARRAY_COMMAND_H

#include <uavcan_ros_bridge.h>
#include <array_command.h>
#include <sam_msgs/msg/array_command.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::ArrayCommand> ros_msg, uavcan_equipment_actuator_ArrayCommand& uav_msg);

}

#endif
