#include <sam_uavcan_bridge/ros_to_uav/thruster_dc_id.h>

namespace ros_to_uav {

template <>
bool convert(const smarc_msgs::ThrusterDC& ros_msg, smarc_uavcan_messages::ThrusterDcID& uav_msg, unsigned char uid)
{
    uav_msg.id = uid;
    uav_msg.thruster.dc = ros_msg.dc;

    return true;
}

}
