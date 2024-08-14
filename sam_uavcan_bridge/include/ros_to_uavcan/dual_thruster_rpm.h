#ifndef ROS_TO_UAV_DUAL_THRUSTER_RPM_H
#define ROS_TO_UAV_DUAL_THRUSTER_RPM_H

#include <uavcan_ros_bridge.h>
#include <smarc_msgs/msg/dual_thruster_rpm.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<smarc_msgs::msg::DualThrusterRPM> ros_msg, smarc_uavcan_messages_DualThrusterRPM& uav_msg );

}

#endif
