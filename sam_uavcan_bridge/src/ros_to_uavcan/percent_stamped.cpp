#include <percent_stamped.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::PercentStamped> ros_msg, uavcan_equipment_actuator_ArrayCommand& uav_msg, unsigned char uid)
{

    // uav_msg.commands.resize(1);
    uav_msg.commands.data[0].actuator_id = uid;
    uav_msg.commands.data[0].command_value = ros_msg->value;
    uav_msg.commands.data[0].command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION;

    return true;
}

}
