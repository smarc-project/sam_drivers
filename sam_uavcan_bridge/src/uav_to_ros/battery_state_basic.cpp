#include <sam_uavcan_bridge/uav_to_ros/battery_state_basic.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::BatteryStateBasic& uav_msg, sensor_msgs::BatteryState& ros_msg)
{
    ros_msg.header.stamp = convert_timestamp(uav_msg.timestamp);
    ros_msg.voltage = uav_msg.voltage;
    ros_msg.current = uav_msg.current;
    ros_msg.charge = uav_msg.charge;
    ros_msg.capacity = uav_msg.capacity;
    ros_msg.design_capacity = uav_msg.design_capacity;
    ros_msg.percentage = uav_msg.percentage;
    ros_msg.power_supply_status = uav_msg.power_supply_status;
    ros_msg.power_supply_health = uav_msg.power_supply_health;

    return true;
}

}
