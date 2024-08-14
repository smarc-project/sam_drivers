#include <ros_to_uavcan/dvl.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<std_msgs::msg::Bool> ros_msg, uavcan_equipment_actuator_ArrayCommand& uav_msg, unsigned char uid, DVLTag)
{
    const bool boolean = ros_msg->data;
    uav_msg.commands.data[0].actuator_id = uid;
    uav_msg.commands.data[0].command_value = boolean;
    uav_msg.commands.data[0].command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION;
    return true;

}

}
