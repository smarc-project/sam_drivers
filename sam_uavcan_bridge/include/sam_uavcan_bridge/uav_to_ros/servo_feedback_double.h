#ifndef UAV_TO_ROS_SENSOR_PRESSURE_STAMPED_H
#define UAV_TO_ROS_SENSOR_PRESSURE_STAMPED_H

#include <smarc_uavcan_messages/ServoFeedbackDouble.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sensor_msgs/JointState.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<smarc_uavcan_messages::ServoFeedbackDouble>& uav_msg, sensor_msgs::JointState& ros_msg);

}

#endif
