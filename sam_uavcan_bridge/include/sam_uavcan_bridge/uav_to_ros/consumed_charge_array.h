#ifndef UAV_TO_ROS_CONSUMED_CHARGE_ARRAY_H
#define UAV_TO_ROS_CONSUMED_CHARGE_ARRAY_H

#include <smarc_uavcan_messages/ConsumedChargeArray.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/ConsumedChargeArray.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::ConsumedChargeArray& uav_msg, sam_msgs::ConsumedChargeArray& ros_msg);

}

#endif
