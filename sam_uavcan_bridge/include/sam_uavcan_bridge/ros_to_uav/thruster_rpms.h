#ifndef ROS_TO_UAV_THRUSTER_RPMS_H
#define ROS_TO_UAV_THRUSTER_RPMS_H

#include <uavcan/equipment/esc/RPMCommand.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/ThrusterRPMs.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::ThrusterRPMs& ros_msg, uavcan::equipment::esc::RPMCommand& uav_msg);

}

#endif
