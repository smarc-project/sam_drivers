#ifndef UAVCAN_UPDATE_BATTERY_H
#define UAVCAN_UPDATE_BATTERY_H

#include <uavcan_ros_bridge.h>
#include <sam_msgs/srv/uavcan_update_battery.hpp>

namespace ros_to_uav {

template <>
bool convert_request(const std::shared_ptr<sam_msgs::srv::UavcanUpdateBattery::Request> ros_request, smarc_uavcan_services_UpdateBatteryRequest& uav_request);

template <>
bool convert_response(const smarc_uavcan_services_UpdateBatteryResponse& uav_response, std::shared_ptr<sam_msgs::srv::UavcanUpdateBattery::Response> ros_response);

}

#endif