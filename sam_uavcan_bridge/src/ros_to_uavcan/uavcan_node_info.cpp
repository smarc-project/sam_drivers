#include <uavcan_node_info.h>
#include <iostream> // Include the necessary header for logging

namespace ros_to_uav {

template <>
bool convert_request(const std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetNodeInfo::Request> ros_request, uavcan_protocol_GetNodeInfoRequest& uav_request)
{
    std::cout << "Converting ROS request to UAVCAN request..." << std::endl;

    return true;
}

template <>
bool convert_response(const uavcan_protocol_GetNodeInfoResponse& uav_response, std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetNodeInfo::Response> ros_response)
{
    std::cout << "Converting UAVCAN response to ROS response..." << std::endl;

    size_t name_size = 0;
    while (name_size < 80 && uav_response.name.data[name_size] != 0) {
        name_size++;
    }

    ros_response->name.resize(name_size);
    for (size_t i = 0; i < name_size; ++i) {
        ros_response->name[i] = uav_response.name.data[i];
    }

    ros_response->status.uptime_sec = uav_response.status.uptime_sec;
    ros_response ->status.health = static_cast<uint8_t>(uav_response.status.health & 0x03);  
    ros_response->status.mode = static_cast<uint8_t>(uav_response.status.mode & 0x07);  
    ros_response->status.sub_mode = static_cast<uint8_t>(uav_response.status.sub_mode & 0x07); 
    ros_response->status.vendor_specific_status_code = uav_response.status.vendor_specific_status_code;
    return true;
}

}
