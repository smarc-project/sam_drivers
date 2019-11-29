#include <sam_uavcan_bridge/uav_to_ros/temperature.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<uavcan::equipment::device::Temperature>& uav_msg, sensor_msgs::Temperature& ros_msg, unsigned char uid)
{
    if (uav_msg.device_id != uid) {
        return false;
    }

    ros_msg.header.stamp = ros::Time::now();
    ros_msg.temperature = uav_msg.temperature;
    ros_msg.variance = 0;

    return true;
}

}
