#include <sam_uavcan_bridge/ros_to_uav/thruster_rpm_id.h>

namespace ros_to_uav {

template <>
bool convert(const smarc_msgs::ThrusterRPM& ros_msg, smarc_uavcan_messages::ThrusterRpmID& uav_msg, unsigned char uid)
{
    uav_msg.id = uid;
    uav_msg.thruster.rpm = ros_msg.rpm;

    return true;
}

}
