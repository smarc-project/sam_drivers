#include <sam_uavcan_bridge/ros_to_uav/thruster_rpm.h>

namespace ros_to_uav {

template <>
bool convert(const smarc_msgs::ThrusterRPM& ros_msg, uavcan::equipment::esc::RPMCommand& uav_msg, unsigned char uid)
{
    uav_msg.rpm.resize(uid+1);
    uav_msg.rpm[uid] = ros_msg.rpm;
    if (0 < uid)
    {
        for (int i = 0; i < uid; i++)
        {
            uav_msg.rpm[i] = 130000;
        }
    }
    return true;
}

}
