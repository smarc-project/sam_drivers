#ifndef UAV_TO_ROS_ESC_STATUS_H
#define UAV_TO_ROS_ESC_STATUS_H

#include <uavcan_ros_bridge.h>
#include <uavcan_ros_msgs/msg/esc_status.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_esc_Status& uav_msg, std::shared_ptr<uavcan_ros_msgs::msg::ESCStatus> ros_msg, unsigned char uid);

}

#endif
