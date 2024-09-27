#ifndef UAV_TO_ROS_TEMPERATURE_H
#define UAV_TO_ROS_TEMPERATURE_H

#include <uavcan_ros_bridge.h>
#include <sensor_msgs/msg/temperature.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_device_Temperature& uav_msg, std::shared_ptr<sensor_msgs::msg::Temperature> ros_msg, unsigned char uid);

}

#endif
