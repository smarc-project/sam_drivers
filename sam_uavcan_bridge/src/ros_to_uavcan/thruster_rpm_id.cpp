#include <thruster_rpm_id.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<smarc_msgs::msg::ThrusterRPM> ros_msg, smarc_uavcan_messages_ThrusterRpmID& uav_msg, unsigned char uid,DefaultTag)
{
    uav_msg.id = uid;
    uav_msg.thruster.rpm = ros_msg->rpm;

    return true;
}

}
