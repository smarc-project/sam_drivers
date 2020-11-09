#include <sam_uavcan_bridge/uav_to_ros/consumed_charge_array.h>

namespace uav_to_ros {

template <>
bool convert(const smarc_uavcan_messages::ConsumedChargeArray& uav_msg, sam_msgs::ConsumedChargeArray& ros_msg)
{
    ros_msg.header.stamp = convert_timestamp(uav_msg.timestamp);

    for(auto c:uav_msg.array)
    {
        sam_msgs::ConsumedCharge msg;
        msg.header.stamp = ros::Time::now();
        msg.circuit_id = c.circuit_id;
        msg.charge = c.charge;
        ros_msg.array.push_back(msg);
    }

    return true;
}

}
