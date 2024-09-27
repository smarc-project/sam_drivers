#ifndef ROS_TO_UAV_BALLAST_ANGLES_H
#define ROS_TO_UAV_BALLAST_ANGLES_H

#include "uavcan_ros_bridge.h"
#include "sam_msgs/msg/ballast_angles.hpp"

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::BallastAngles> ros_msg, uavcan_equipment_actuator_ArrayCommand& uav_msg, unsigned char uid,DefaultTag);

}

#endif
