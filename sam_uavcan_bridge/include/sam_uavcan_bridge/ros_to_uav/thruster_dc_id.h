#ifndef ROS_TO_UAV_THRUSTER_DC_ID_H
#define ROS_TO_UAV_THRUSTER_DC_ID_H

#include <smarc_uavcan_messages/ThrusterDcID.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <smarc_msgs/ThrusterDC.h>

namespace ros_to_uav {

template <>
bool convert(const smarc_msgs::ThrusterDC& ros_msg, smarc_uavcan_messages::ThrusterDcID& uav_msg, unsigned char uid);

}

#endif
