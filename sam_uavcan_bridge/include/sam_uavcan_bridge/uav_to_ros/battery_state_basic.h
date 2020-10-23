#ifndef UAV_TO_ROS_BATTERY_STATE_BASIC_H
#define UAV_TO_ROS_BATTERY_STATE_BASIC_H

#include <smarc_uavcan_messages/BatteryStateBasic.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/BatteryState.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::BatteryStateBasic& uav_msg, sensor_msgs::BatteryState& ros_msg);
// bool convert(const uavcan::ReceivedDataStructure<smarc_uavcan_messages::BatteryStateBasic>& uav_msg, sensor_msgs::BatteryState& ros_msg);

}

#endif
