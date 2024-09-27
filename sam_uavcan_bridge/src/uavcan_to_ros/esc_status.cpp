#include <uavcan_to_ros/esc_status.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_esc_Status& uav_msg, std::shared_ptr<uavcan_ros_msgs::msg::ESCStatus> ros_msg, unsigned char uid)
{
    if (uid != 255 && uav_msg.esc_index != uid) {
        return false;
    }

    ros_msg->error_count = uav_msg.error_count; 
    ros_msg->voltage = uav_msg.voltage;
    ros_msg->current = uav_msg.current;
    ros_msg->temperature = uav_msg.temperature;
    ros_msg->rpm = uav_msg.rpm;
    ros_msg->power_rating_pct = uav_msg.power_rating_pct;
    ros_msg->esc_index = uav_msg.esc_index;

    return true;
}

}
