#include <pressure.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_air_data_StaticPressure& uav_msg, std::shared_ptr<sensor_msgs::msg::FluidPressure> ros_msg)
{
    ros_msg->header.stamp = rclcpp::Clock().now();
    ros_msg->fluid_pressure = uav_msg.static_pressure;
    ros_msg->variance = uav_msg.static_pressure_variance;
    return true;
}

}
