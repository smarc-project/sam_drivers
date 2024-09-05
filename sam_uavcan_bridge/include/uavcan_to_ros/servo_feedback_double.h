#ifndef UAV_TO_ROS_SERVO_FEEDBACK_DOUBLE_H
#define UAV_TO_ROS_SERVO_FEEDBACK_DOUBLE_H

#include <uavcan_ros_bridge.h>
#include <sensor_msgs/msg/joint_state.hpp>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_ServoFeedbackDouble& uav_msg, std::shared_ptr<sensor_msgs::msg::JointState> ros_msg);

}

#endif
