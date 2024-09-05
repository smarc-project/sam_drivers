#include <uavcan_to_ros/consumed_charge_array.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages_ConsumedChargeArray& uav_msg, std::shared_ptr<sam_msgs::msg::ConsumedChargeArray> ros_msg)
{
    ros_msg->header.stamp = convert_timestamp(uav_msg.timestamp.usec);

    for(auto c:uav_msg.array.data)
    {
        sam_msgs::msg::ConsumedCharge msg;
        msg.header.stamp = rclcpp::Clock().now();
        msg.circuit_id = c.circuit_id;
        msg.charge = c.charge;
        ros_msg->array.push_back(msg);
    }

    return true;
}

}
