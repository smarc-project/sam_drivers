#ifndef UAVCAN_TRANSPORT_STATS_H
#define UAVCAN_TRANSPORT_STATS_H

#include <uavcan_ros_bridge.h>
#include <uavcan_ros_msgs/srv/uavcan_get_transport_stats.hpp>

namespace ros_to_uav {

template <>
bool convert_request(const std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetTransportStats::Request> ros_request, uavcan_protocol_GetTransportStatsRequest& uav_request);

template <>
bool convert_response(const uavcan_protocol_GetTransportStatsResponse& uav_response, std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetTransportStats::Response> ros_response);

}

#endif
