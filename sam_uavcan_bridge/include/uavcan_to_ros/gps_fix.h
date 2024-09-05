#ifndef UAV_TO_ROS_GPS_FIX_H
#define UAV_TO_ROS_GPS_FIX_H

#include <uavcan_ros_bridge.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_gnss_Fix& uav_msg, std::shared_ptr<sensor_msgs::msg::NavSatFix> ros_msg);

}

#endif // UAV_TO_ROS_GPS_FIX_H
