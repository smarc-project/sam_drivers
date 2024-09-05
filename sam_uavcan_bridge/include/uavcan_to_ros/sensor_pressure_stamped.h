#ifndef UAV_TO_ROS_SENSOR_PRESSURE_STAMPED_H
#define UAV_TO_ROS_SENSOR_PRESSURE_STAMPED_H

#include <uavcan_ros_bridge.h>
#include <sensor_msgs/msg/fluid_pressure.hpp>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_SensorPressureStamped& uav_msg, std::shared_ptr<sensor_msgs::msg::FluidPressure> ros_msg, unsigned char uid);

}

#endif
