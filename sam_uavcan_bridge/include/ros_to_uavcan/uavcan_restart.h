#ifndef UAVCAN_RESTART_H
#define UAVCAN_RESTART_H

#include <uavcan_ros_bridge.h>
#include <uavcan_ros_msgs/srv/uavcan_restart_node.hpp>

namespace ros_to_uav {

template <>
bool convert_request(const std::shared_ptr<uavcan_ros_msgs::srv::UavcanRestartNode::Request> ros_request, uavcan_protocol_RestartNodeRequest& uav_request);

template <>
bool convert_response(const uavcan_protocol_RestartNodeResponse& uav_response, std::shared_ptr<uavcan_ros_msgs::srv::UavcanRestartNode::Response> ros_response);

}

#endif
