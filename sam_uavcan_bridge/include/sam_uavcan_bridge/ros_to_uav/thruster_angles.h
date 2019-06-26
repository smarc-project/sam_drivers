#ifndef ROS_TO_UAV_THRUSTER_ANGLES_H
#define ROS_TO_UAV_THRUSTER_ANGLES_H

#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/ThrusterAngles.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::ThrusterAngles& ros_msg, uavcan::equipment::actuator::ArrayCommand& uav_msg, unsigned char uid);

}

#endif
