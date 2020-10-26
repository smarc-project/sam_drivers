#ifndef ROS_TO_UAV_DUAL_THRUSTER_RPM_H
#define ROS_TO_UAV_DUAL_THRUSTER_RPM_H

#include <smarc_uavcan_messages/DualThrusterRPM.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <smarc_msgs/DualThrusterRPM.h>

namespace ros_to_uav {

template <>
bool convert(const smarc_msgs::DualThrusterRPM& ros_msg, smarc_uavcan_messages::DualThrusterRPM& uav_msg);

}

#endif
