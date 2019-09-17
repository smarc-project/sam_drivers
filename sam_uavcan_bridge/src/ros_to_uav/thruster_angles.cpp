#include <sam_uavcan_bridge/ros_to_uav/thruster_angles.h>

namespace ros_to_uav {

template <>
bool convert(const sam_msgs::ThrusterAngles& ros_msg, uavcan::equipment::actuator::ArrayCommand& uav_msg, unsigned char uid)
{
    uav_msg.commands.resize(2);

    // 16 is the elevator, 17 is the rudder
    uav_msg.commands[0].actuator_id = uid;
    uav_msg.commands[0].command_value = ros_msg.thruster_vertical_radians;
    uav_msg.commands[0].command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_POSITION;

    uav_msg.commands[1].actuator_id = uid + 1;
    uav_msg.commands[1].command_value = ros_msg.thruster_horizontal_radians;
    uav_msg.commands[1].command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_POSITION;

    return true;
}

}
