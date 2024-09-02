#include <uavcan_to_ros/panic.h>
#include <string>
namespace uav_to_ros {

template <>
bool convert(const uavcan_protocol_Panic& uav_msg, std::shared_ptr<std_msgs::msg::String> ros_msg)
{
    std::string reason;

    for (size_t i = 0; i < sizeof(uav_msg.reason_text); ++i) {
        if (uav_msg.reason_text.data[i] == '\0') {
            break;
        }
        reason += static_cast<char>(uav_msg.reason_text.data[i]);
    }
    ros_msg->data = reason;

    return true;
}

}
