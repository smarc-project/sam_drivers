#include <sss.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<std_msgs::msg::Bool> ros_msg,uavcan_equipment_actuator_ArrayCommand& uav_msg, unsigned char uid)
{
    // const std::string sub = ros_msg.data.substr(0, 7);
    // const char* c = sub.c_str();
    // uav_msg.reason_text = c;

    const bool boolean = ros_msg->data;//.substr(0, 7);
    // const char* c = sub.c_str();

    // uav_msg.commands.resize(1);
    
    uav_msg.commands.data[0].actuator_id = uid;
    uav_msg.commands.data[0].command_value = boolean;//c;
    uav_msg.commands.data[0].command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION;
    return true;
}

}
