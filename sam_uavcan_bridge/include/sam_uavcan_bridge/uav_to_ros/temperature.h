#ifndef UAV_TO_ROS_TEMPERATURE_H
#define UAV_TO_ROS_TEMPERATURE_H

#include <uavcan/equipment/device/Temperature.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/Temperature.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<uavcan::equipment::device::Temperature>& uav_msg, sensor_msgs::Temperature& ros_msg, unsigned char uid);

}

#endif
