#ifndef UAV_TO_ROS_CONSUMED_CHARGE_ARRAY_H
#define UAV_TO_ROS_CONSUMED_CHARGE_ARRAY_H

#include <uavcan_ros_bridge.h>
#include <sam_msgs/msg/consumed_charge_array.hpp>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_ConsumedChargeArray& uav_msg, std::shared_ptr<sam_msgs::msg::ConsumedChargeArray> ros_msg);

}

#endif
