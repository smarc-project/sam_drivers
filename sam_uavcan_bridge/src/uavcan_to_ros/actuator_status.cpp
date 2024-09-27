#include <actuator_status.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_actuator_Status& uav_msg, std::shared_ptr<sam_msgs::msg::PercentStamped> ros_msg, unsigned char uid)
{
    if (uav_msg.actuator_id != uid) {
        return false;
    }

    ros_msg->header.stamp = rclcpp::Clock().now();
    ros_msg->value = uav_msg.position;

    return true;
}

}
