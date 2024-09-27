#include <uavcan_to_ros/circuit_status_stamped.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_power_CircuitStatus& uav_msg, std::shared_ptr<sam_msgs::msg::CircuitStatusStamped> ros_msg)
{
    ros_msg->header.stamp = rclcpp::Clock().now();
    ros_msg->circuit.error_flags = uav_msg.error_flags;
    ros_msg->circuit.circuit_id = uav_msg.circuit_id;
    ros_msg->circuit.voltage = uav_msg.voltage;
    ros_msg->circuit.current = uav_msg.current;

    return true;
}

}
