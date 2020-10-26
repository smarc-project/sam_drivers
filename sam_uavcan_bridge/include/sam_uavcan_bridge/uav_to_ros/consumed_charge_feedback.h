#ifndef UAV_TO_ROS_CONSUMED_CHARGE_FEEDBACK_H
#define UAV_TO_ROS_CONSUMED_CHARGE_FEEDBACK_H

#include <smarc_uavcan_messages/ConsumedChargeFeedback.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/ConsumedChargeFeedback.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::ConsumedChargeFeedback& uav_msg, sam_msgs::ConsumedChargeFeedback& ros_msg);

}

#endif
