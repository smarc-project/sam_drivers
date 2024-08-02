#include <light_command.h>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::LightCommand> ros_msg, uavcan_equipment_indication_LightsCommand& uav_msg)
{
    uavcan_equipment_indication_SingleLightCommand msg;
    msg.light_id = ros_msg->id;
    unsigned r = ros_msg->command.a * ros_msg->command.r;
    unsigned g = ros_msg->command.a * ros_msg->command.g;
    unsigned b = ros_msg->command.a * ros_msg->command.b;
    msg.color.red = r >> 3;
    msg.color.green = g >> 2;
    msg.color.blue = b >> 3;
    // uav_msg.commands.push_back(msg);
    uav_msg.commands.data[0] = msg;

    return true;
}

}
