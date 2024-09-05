#ifndef UAV_TO_ROS_LEAK_H
#define UAV_TO_ROS_LEAK_H

#include <uavcan_ros_bridge.h>
#include <sam_msgs/msg/leak.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_actuator_Status& uav_msg, std::shared_ptr<sam_msgs::msg::Leak> ros_msg, unsigned char uid);

}

#endif
