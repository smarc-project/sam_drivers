#ifndef ROS_TO_UAV_PERCENT_STAMPED_H
#define ROS_TO_UAV_PERCENT_STAMPED_H

#include <uavcan_ros_bridge.h>
#include <sam_msgs/msg/percent_stamped.hpp>

namespace ros_to_uav {

template <>
bool convert(const std::shared_ptr<sam_msgs::msg::PercentStamped> ros_msg, uavcan_equipment_actuator_ArrayCommand& uav_msg, unsigned char uid);

}

#endif
