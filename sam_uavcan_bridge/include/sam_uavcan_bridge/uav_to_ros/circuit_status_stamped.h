#ifndef UAV_TO_ROS_CIRCUIT_STATUS_STAMPED_H
#define UAV_TO_ROS_CIRCUIT_STATUS_STAMPED_H

#include <uavcan/equipment/power/CircuitStatus.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/CircuitStatusStamped.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::power::CircuitStatus& uav_msg, sam_msgs::CircuitStatusStamped& ros_msg);

}

#endif
