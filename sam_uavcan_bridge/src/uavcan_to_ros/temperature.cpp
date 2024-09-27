#include <uavcan_to_ros/temperature.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_device_Temperature& uav_msg, std::shared_ptr<sensor_msgs::msg::Temperature> ros_msg, unsigned char uid)
{
    if (uav_msg.device_id != uid) {
        return false;
    }

    ros_msg->header.stamp = rclcpp::Clock().now();
    ros_msg->temperature = uav_msg.temperature;
    ros_msg->variance = 0;
    return true;
}

}
