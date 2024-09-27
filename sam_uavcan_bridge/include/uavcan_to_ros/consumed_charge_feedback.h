#ifndef UAV_TO_ROS_CONSUMED_CHARGE_FEEDBACK_H
#define UAV_TO_ROS_CONSUMED_CHARGE_FEEDBACK_H

#include <uavcan_ros_bridge.h>
#include <sam_msgs/msg/consumed_charge_feedback.hpp>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_ConsumedChargeFeedback& uav_msg, std::shared_ptr<sam_msgs::msg::ConsumedChargeFeedback> ros_msg);

}

#endif
