#ifndef ROS_TO_UAV_THRUSTER_RPM_ID_H
#define ROS_TO_UAV_THRUSTER_RPM_ID_H

#include <smarc_uavcan_messages/ThrusterRpmID.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <smarc_msgs/ThrusterRPM.h>

namespace ros_to_uav {

template <>
bool convert(const smarc_msgs::ThrusterRPM& ros_msg, smarc_uavcan_messages::ThrusterRpmID& uav_msg, unsigned char uid);

}

#endif
