#include <uavcan_to_ros/ctd_feedback.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_CTDFeedback& uav_msg, std::shared_ptr<smarc_msgs::msg::CTD> ros_msg)
{
    ros_msg->header.stamp = convert_timestamp(uav_msg.timestamp.usec);
    ros_msg->conductivity = uav_msg.conductivity;
    ros_msg->temperature = uav_msg.temperature;
    ros_msg->depth = uav_msg.depth;

    return true;
}

}
