#include <sam_uavcan_bridge/ros_to_uav/panic.h>

namespace ros_to_uav {

template <>
bool convert(const std_msgs::String& ros_msg, uavcan::protocol::Panic& uav_msg)
{
    const std::string sub = ros_msg.data.substr(0, 7);
    const char * c = sub.c_str();
    uav_msg.reason_text = c;

    return true;
}

}
