#ifndef UAV_TO_ROS_CIRCUIT_STATUS_STAMPED_H
#define UAV_TO_ROS_CIRCUIT_STATUS_STAMPED_H
#include <uavcan_ros_bridge.h>
#include <sam_msgs/msg/circuit_status_stamped.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_power_CircuitStatus& uav_msg, std::shared_ptr<sam_msgs::msg::CircuitStatusStamped> ros_msg);

}

#endif
