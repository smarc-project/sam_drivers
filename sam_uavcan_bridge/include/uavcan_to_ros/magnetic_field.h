#ifndef UAV_TO_ROS_MAGNETIC_FIELD_H
#define UAV_TO_ROS_MAGNETIC_FIELD_H

#include <uavcan_ros_bridge.h>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_ahrs_MagneticFieldStrength& uav_msg, std::shared_ptr<sensor_msgs::msg::MagneticField> ros_msg);

}

#endif // UAV_TO_ROS_MAGNETIC_FIELD_H
