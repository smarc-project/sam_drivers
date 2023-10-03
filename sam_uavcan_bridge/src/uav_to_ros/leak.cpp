#include <sam_uavcan_bridge/uav_to_ros/leak.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::SensorLeak& uav_msg, sam_msgs::Leak& ros_msg)
{
    // if (uav_msg.actuator_id != uid) {
    //     return false;
    // }

    ros_msg.value = uav_msg.leak;
    ros_msg.sensor_id = uav_msg.sensor_id;

    return true;
}

}
