#ifndef UAV_TO_ROS_LEAK_H
#define UAV_TO_ROS_LEAK_H

// #include <uavcan/equipment/actuator/Status.hpp>
#include <smarc_uavcan_messages/SensorLeak.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/Leak.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::SensorLeak& uav_msg, sam_msgs::Leak& ros_msg);

}

#endif
