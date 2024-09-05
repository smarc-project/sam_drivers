#include <uavcan_to_ros/servo_feedback_double.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_ServoFeedbackDouble& uav_msg, std::shared_ptr<sensor_msgs::msg::JointState> ros_msg)
{
    ros_msg->header.stamp = convert_timestamp(uav_msg.timestamp.usec);
    ros_msg->position.push_back(uav_msg.position_1);
    ros_msg->name.push_back("elevator");
    ros_msg->position.push_back(uav_msg.position_2);
    ros_msg->name.push_back("rudder");
    return true;
}

}
