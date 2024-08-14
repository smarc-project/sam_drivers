#include <thruster_rpm.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<smarc_msgs::msg::ThrusterRPM> ros_msg, uavcan_equipment_esc_RPMCommand& uav_msg, unsigned char uid,DefaultTag){
    // uav_msg.rpm.resize(uid+1);
    uav_msg.rpm.data[uid] = ros_msg->rpm;
    if (0 < uid)
    {
        for (int i = 0; i < uid; i++)
        {
            uav_msg.rpm.data[i] = 130000;
        }
    }
    return true;
}

}
