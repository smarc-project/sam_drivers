#ifndef UAV_TO_ROS_BATTERY_INFO_H
#define UAV_TO_ROS_BATTERY_INFO_H

#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/BatteryState.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::power::BatteryInfo& uav_msg, sensor_msgs::BatteryState& ros_msg);

}

#endif
