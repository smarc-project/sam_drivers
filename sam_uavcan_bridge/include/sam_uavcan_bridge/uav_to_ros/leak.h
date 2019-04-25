#ifndef UAV_TO_ROS_LEAK_H
#define UAV_TO_ROS_LEAK_H

#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/Leak.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>& uav_msg, sam_msgs::Leak& ros_msg, unsigned char uid);

}

#endif
