#ifndef ROS_TO_UAV_DROP_H
#define ROS_TO_UAV_DROP_H

#include <uavcan_ros_bridge.h>
#include <std_msgs/msg/bool.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<std_msgs::msg::Bool> ros_msg,uavcan_equipment_actuator_ArrayCommand& uav_msg, unsigned char uid);

}

#endif
