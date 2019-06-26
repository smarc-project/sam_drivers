#include <sam_uavcan_bridge/ros_to_uav/thruster_rpms.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::ThrusterRPMs& ros_msg, uavcan::equipment::esc::RPMCommand& uav_msg)
{
    uav_msg.rpm.resize(2);
    uav_msg.rpm[0] = ros_msg.thruster_1_rpm;
    uav_msg.rpm[1] = ros_msg.thruster_2_rpm;

    return true;
}

}
