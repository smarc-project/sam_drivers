#include <uavcan_to_ros/leak.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_actuator_Status& uav_msg, std::shared_ptr<sam_msgs::msg::Leak> ros_msg, unsigned char uid)
{
    if (uav_msg.actuator_id != uid) {
        return false;
    }

    ros_msg->value = uav_msg.position == 100.;
    ros_msg->leak_counter = uav_msg.power_rating_pct;

    return true;
}

}
