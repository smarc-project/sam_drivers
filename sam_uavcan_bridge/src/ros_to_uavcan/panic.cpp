#include <panic.h>
#include <cstring>
namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<std_msgs::msg::String> ros_msg, uavcan_protocol_Panic& uav_msg)
{
    const std::string sub = ros_msg->data.substr(0, 7);
     std::strncpy(reinterpret_cast<char*>(uav_msg.reason_text.data), sub.c_str(), sizeof(uav_msg.reason_text.data));
    uav_msg.reason_text.len = std::min(sub.size(), sizeof(uav_msg.reason_text.data));

    return true;
}

}
