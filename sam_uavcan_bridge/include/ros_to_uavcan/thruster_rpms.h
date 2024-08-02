#ifndef ROS_TO_UAV_THRUSTER_RPMS_H
#define ROS_TO_UAV_THRUSTER_RPMS_H

#include <uavcan_ros_bridge.h>
#include <sam_msgs/msg/thruster_rp_ms.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::ThrusterRPMs> ros_msg, uavcan_equipment_esc_RPMCommand& uav_msg);

}

#endif
