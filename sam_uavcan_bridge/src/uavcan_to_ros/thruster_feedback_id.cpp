#include <uavcan_to_ros/thruster_feedback_id.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_ThrusterFeedbackID& uav_msg, std::shared_ptr<smarc_msgs::msg::ThrusterFeedback> ros_msg, unsigned char uid)
{
    if (uav_msg.id != uid) {
        return false;
    }

    ros_msg->header.stamp = convert_timestamp(uav_msg.timestamp.usec);
    ros_msg->rpm.rpm = uav_msg.thruster.rpm;
    ros_msg->current = uav_msg.current;
    ros_msg->torque = uav_msg.torque;

    return true;
}

}
