#include <uavcan_to_ros/dual_thruster_feedback.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_DualThrusterFeedback& uav_msg, std::shared_ptr<smarc_msgs::msg::DualThrusterFeedback> ros_msg)
{
    ros_msg->thruster_front.header.stamp = convert_timestamp(uav_msg.thruster_front.timestamp.usec);
    ros_msg->thruster_front.rpm.rpm = uav_msg.thruster_front.rpm.rpm;
    ros_msg->thruster_front.current = uav_msg.thruster_front.current;
    ros_msg->thruster_front.torque = uav_msg.thruster_front.torque;

    ros_msg->thruster_back.header.stamp = convert_timestamp(uav_msg.thruster_back.timestamp.usec);
    ros_msg->thruster_back.rpm.rpm = uav_msg.thruster_back.rpm.rpm;
    ros_msg->thruster_back.current = uav_msg.thruster_back.current;
    ros_msg->thruster_back.torque = uav_msg.thruster_back.torque;

    return true;
}

}
