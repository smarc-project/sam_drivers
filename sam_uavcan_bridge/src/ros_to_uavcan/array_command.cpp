#include <array_command.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::ArrayCommand> ros_msg, uavcan_equipment_actuator_ArrayCommand& uav_msg)
{
    // Ensure we do not exceed the array's bounds
    size_t num_commands = std::min(ros_msg->commands.size(), sizeof(uav_msg.commands.data) / sizeof(uav_msg.commands.data[0]));

    for (size_t i = 0; i < num_commands; ++i) {
        uav_msg.commands.data[i].actuator_id = ros_msg->commands[i].actuator_id;
        uav_msg.commands.data[i].command_value = ros_msg->commands[i].command_value;
        uav_msg.commands.data[i].command_type = ros_msg->commands[i].command_type;
    }

    return true;
}
    
    }