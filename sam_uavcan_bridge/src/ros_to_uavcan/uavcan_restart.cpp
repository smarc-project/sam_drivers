#include <ros_to_uavcan/uavcan_restart.h>

namespace ros_to_uav {

#pragma GCC diagnostic ignored "-Wunused-parameter"
#define UAVCAN_RESTARTNODE_MAGIC_NUMBER 0xACCE551B1E // Taken from the service definition check the dsdl file

template <>
bool convert_request(const std::shared_ptr<uavcan_ros_msgs::srv::UavcanRestartNode::Request> ros_request, uavcan_protocol_RestartNodeRequest& uav_request)

{
    uav_request.magic_number = UAVCAN_RESTARTNODE_MAGIC_NUMBER;
    return true;
}

template <>
bool convert_response(const uavcan_protocol_RestartNodeResponse& uav_response, std::shared_ptr<uavcan_ros_msgs::srv::UavcanRestartNode::Response> ros_response)
{
    return true;
}
#pragma GCC diagnostic pop

}
