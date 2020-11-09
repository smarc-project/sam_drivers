#include <sam_uavcan_bridge/ros_to_uav/light_command.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::LightCommand& ros_msg, uavcan::equipment::indication::LightsCommand& uav_msg)
{
    uavcan::equipment::indication::SingleLightCommand msg;
    msg.light_id = ros_msg.id;
    unsigned r = ros_msg.command.a * ros_msg.command.r;
    unsigned g = ros_msg.command.a * ros_msg.command.g;
    unsigned b = ros_msg.command.a * ros_msg.command.b;
    msg.color.red = r >> 3;
    msg.color.green = g >> 2;
    msg.color.blue = b >> 3;
    uav_msg.commands.push_back(msg);

    return true;
}

}
