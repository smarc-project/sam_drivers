#ifndef ROS_TO_UAV_COMMAND_H
#define ROS_TO_UAV_COMMAND_H


#include <uavcan_ros_bridge.h>
#include <std_msgs/msg/float32.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<std_msgs::msg::Float32>ros_msg, uavcan_equipment_actuator_ArrayCommand& uav_msg);

}

#endif
