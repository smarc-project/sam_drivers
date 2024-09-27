#include <ros_to_uavcan/uavcan_transport_stats.h>

namespace ros_to_uav {

template <>
bool convert_request(const std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetTransportStats::Request> ros_request, uavcan_protocol_GetTransportStatsRequest& uav_request)
{
    return true;
}

template <>
bool convert_response(const uavcan_protocol_GetTransportStatsResponse& uav_response, std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetTransportStats::Response> ros_response)
{
    ros_response->transfers_tx = uav_response.transfers_tx;
    ros_response->transfers_rx = uav_response.transfers_rx;
    ros_response->transfer_errors = uav_response.transfer_errors;

    return true;
}

}
