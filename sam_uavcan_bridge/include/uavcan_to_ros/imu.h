#ifndef UAV_TO_ROS_IMU_H
#define UAV_TO_ROS_IMU_H

#include <uavcan_ros_bridge.h>
#include <sensor_msgs/msg/imu.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_ahrs_Solution& uav_msg, std::shared_ptr<sensor_msgs::msg::Imu> ros_msg);

}

#endif
