#include <sam_uavcan_bridge/ros_to_uav/dual_thruster_rpm.h>

namespace ros_to_uav {

template <>
bool convert(const smarc_msgs::DualThrusterRPM& ros_msg, smarc_uavcan_messages::DualThrusterRPM& uav_msg)
{
    uav_msg.thruster_front.rpm = ros_msg.thruster_front.rpm;
    uav_msg.thruster_back.rpm = ros_msg.thruster_back.rpm;

    return true;
}

}
