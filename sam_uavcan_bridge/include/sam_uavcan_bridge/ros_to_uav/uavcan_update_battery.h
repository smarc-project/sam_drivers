#ifndef UAVCAN_UPDATE_BATTERY_H
#define UAVCAN_UPDATE_BATTERY_H

#include <smarc_uavcan_services/UpdateBattery.hpp>
#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <sam_msgs/UavcanUpdateBattery.h>

namespace ros_to_uav {

template <>
bool convert_request(const sam_msgs::UavcanUpdateBattery::Request& ros_request, smarc_uavcan_services::UpdateBattery::Request& uav_request);

template <>
bool convert_response(const smarc_uavcan_services::UpdateBattery::Response& uav_response, sam_msgs::UavcanUpdateBattery::Response& ros_response);

}

#endif