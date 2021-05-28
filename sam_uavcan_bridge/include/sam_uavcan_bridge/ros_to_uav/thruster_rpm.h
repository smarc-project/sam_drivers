#ifndef ROS_TO_UAV_THRUSTER_RPM_H
#define ROS_TO_UAV_THRUSTER_RPM_H

#include <uavcan/equipment/esc/RPMCommand.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <smarc_msgs/ThrusterRPM.h>

namespace ros_to_uav {

template <>
bool convert(const smarc_msgs::ThrusterRPM& ros_msg, uavcan::equipment::esc::RPMCommand& uav_msg, unsigned char uid);

}

#endif
