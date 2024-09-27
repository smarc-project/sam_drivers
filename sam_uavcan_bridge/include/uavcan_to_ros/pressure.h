#ifndef UAV_TO_ROS_PRESSURE_H
#define UAV_TO_ROS_PRESSURE_H

#include <uavcan_ros_bridge.h>
#include <sensor_msgs/msg/fluid_pressure.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_air_data_StaticPressure& uav_msg, std::shared_ptr<sensor_msgs::msg::FluidPressure> ros_msg);

}

#endif
