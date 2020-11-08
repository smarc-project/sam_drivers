#include <sam_uavcan_bridge/uav_to_ros/circuit_status_stamped.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::equipment::power::CircuitStatus& uav_msg, sam_msgs::CircuitStatusStamped& ros_msg)
{
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.circuit.error_flags = uav_msg.error_flags;
    ros_msg.circuit.circuit_id = uav_msg.circuit_id;
    ros_msg.circuit.voltage = uav_msg.voltage;
    ros_msg.circuit.current = uav_msg.current;

    return true;
}

}
