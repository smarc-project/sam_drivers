#ifndef UAVCAN_NODE_INFO_H
#define UAVCAN_NODE_INFO_H

#include <uavcan.protocol.GetNodeInfo_req.h>
#include <uavcan.protocol.GetNodeInfo_res.h>
#include <uavcan_ros_bridge.h>
#include <uavcan_ros_msgs/srv/uavcan_get_node_info.hpp>

namespace ros_to_uav {

template <>
bool convert_request(const std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetNodeInfo::Request> ros_request, uavcan_protocol_GetNodeInfoRequest& uav_request);

template <>
bool convert_response(const uavcan_protocol_GetNodeInfoResponse& uav_response, std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetNodeInfo::Response> ros_response);

}

#endif
