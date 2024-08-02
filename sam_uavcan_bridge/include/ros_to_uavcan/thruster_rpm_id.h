#ifndef ROS_TO_UAV_THRUSTER_RPM_ID_H
#define ROS_TO_UAV_THRUSTER_RPM_ID_H

#include <uavcan_ros_bridge.h>
#include <smarc_msgs/msg/thruster_rpm.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<smarc_msgs::msg::ThrusterRPM> ros_msg, smarc_uavcan_messages_ThrusterRpmID& uav_msg, unsigned char uid);

}

#endif
