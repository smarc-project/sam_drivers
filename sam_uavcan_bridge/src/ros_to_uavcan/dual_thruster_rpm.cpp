#include <dual_thruster_rpm.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<smarc_msgs::msg::DualThrusterRPM> ros_msg, smarc_uavcan_messages_DualThrusterRPM& uav_msg)
{
    uav_msg.thruster_front.rpm = ros_msg->thruster_front.rpm;
    uav_msg.thruster_back.rpm = ros_msg->thruster_back.rpm;

    return true;
}

}
