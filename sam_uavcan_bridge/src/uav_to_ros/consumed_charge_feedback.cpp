#include <sam_uavcan_bridge/uav_to_ros/consumed_charge_feedback.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::ConsumedChargeFeedback& uav_msg, sam_msgs::ConsumedChargeFeedback& ros_msg)
{
    ros_msg.header.stamp = convert_timestamp(uav_msg.timestamp);
    ros_msg.main = uav_msg.main;
    ros_msg.esc1 = uav_msg.esc1;
    ros_msg.esc2 = uav_msg.esc2;
    ros_msg.esc3 = uav_msg.esc3;
    ros_msg.v20 = uav_msg.v20;
    ros_msg.v12 = uav_msg.v12;
    ros_msg.v7 = uav_msg.v7;
    ros_msg.v5 = uav_msg.v5;
    ros_msg.v33 = uav_msg.v33;

    return true;
}

}
