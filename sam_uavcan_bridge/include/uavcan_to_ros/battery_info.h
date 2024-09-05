#ifndef UAV_TO_ROS_BATTERY_INFO_H
#define UAV_TO_ROS_BATTERY_INFO_H

#include <uavcan_ros_bridge.h>
#include <sensor_msgs/msg/battery_state.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_power_BatteryInfo& uav_msg, std::shared_ptr<sensor_msgs::msg::BatteryState> ros_msg);

}

#endif
