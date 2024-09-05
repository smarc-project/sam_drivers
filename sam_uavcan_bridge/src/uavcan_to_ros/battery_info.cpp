#include <uavcan_to_ros/battery_info.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_power_BatteryInfo& uav_msg, std::shared_ptr<sensor_msgs::msg::BatteryState> ros_msg)
{
    ros_msg->header.stamp = rclcpp::Clock().now();
    ros_msg->voltage = uav_msg.voltage;
    ros_msg->current = uav_msg.current;
    // ros_msg.charge = uav_msg.charge;
    // ros_msg.capacity = uav_msg.full_charge_capacity_wh/22.2;
    // ros_msg.design_capacity = uav_msg.design_capacity;
    ros_msg->percentage = uav_msg.state_of_charge_pct/100.0;
    // ros_msg.power_supply_status = uav_msg.power_supply_status;
    // ros_msg.power_supply_health = uav_msg.power_supply_health;

    return true;
}

}
