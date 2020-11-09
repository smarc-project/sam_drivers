#include <sam_uavcan_bridge/uav_to_ros/panic.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::protocol::Panic& uav_msg, std_msgs::String& ros_msg)
{
    const std::string reason = uav_msg.reason_text.c_str();
    ros_msg.data = reason;

    return true;
}

}
