#include "thruster_angles.h"

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::ThrusterAngles> ros_msg, uavcan_equipment_actuator_ArrayCommand& uav_msg, unsigned char uid)
{  
    uav_msg.commands.data[0].actuator_id = uid;
    uav_msg.commands.data[0].command_value = ros_msg->thruster_vertical_radians;
    uav_msg.commands.data[0].command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION;


    uav_msg.commands.data[1].actuator_id = uid + 1;
    uav_msg.commands.data[1].command_value = ros_msg->thruster_horizontal_radians;
    uav_msg.commands.data[1].command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION;

    return true;
}

}
