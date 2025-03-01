#ifndef UAV_TO_ROS_SENSOR_PRESSURE_STAMPED_H
#define UAV_TO_ROS_SENSOR_PRESSURE_STAMPED_H

#include <smarc_uavcan_messages/SensorPressureStamped.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/FluidPressure.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<smarc_uavcan_messages::SensorPressureStamped>& uav_msg, sensor_msgs::FluidPressure& ros_msg, unsigned char uid);

}

#endif
