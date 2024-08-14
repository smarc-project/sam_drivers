#include <command.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<std_msgs::msg::Float32> ros_msg, uavcan_equipment_actuator_ArrayCommand& uav_msg)
{
    // uav_msg.commands.resize(1);
    uav_msg.commands.data[0].command_value = ros_msg->data;
    return true;
}

}
