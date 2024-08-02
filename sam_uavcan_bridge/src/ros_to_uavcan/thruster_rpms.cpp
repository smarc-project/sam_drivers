#include <thruster_rpms.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::ThrusterRPMs> ros_msg, uavcan_equipment_esc_RPMCommand& uav_msg)
{
    // uav_msg.rpm.resize(2);
    uav_msg.rpm.data[0] = ros_msg->thruster_1_rpm;
    uav_msg.rpm.data[1] = ros_msg->thruster_2_rpm;

    return true;
}

}
