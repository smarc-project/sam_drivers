#include <uavcan_to_ros/sensor_pressure.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_SensorPressure& uav_msg, std::shared_ptr<sensor_msgs::msg::FluidPressure> ros_msg, unsigned char uid)
{
    if (uid != 255 && uav_msg.device_id != uid) {
        return false;
    }

    ros_msg->header.stamp = rclcpp::Clock().now();
    ros_msg->fluid_pressure = uav_msg.pressure.static_pressure;
    ros_msg->variance = uav_msg.pressure.static_pressure_variance;
    return true;
}

}
