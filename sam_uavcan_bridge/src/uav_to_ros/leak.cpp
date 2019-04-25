#include <sam_uavcan_bridge/uav_to_ros/leak.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::Status>& uav_msg, sam_msgs::Leak& ros_msg, unsigned char uid)
{
    if (uav_msg.actuator_id != uid) {
        return false;
    }

    ros_msg.value = uav_msg.position == 100.;
    ros_msg.leak_counter = uav_msg.power_rating_pct;

    return true;
}

}
