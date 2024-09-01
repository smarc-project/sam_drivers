#include <uavcan_node_info.h>
#include <iostream> // Include the necessary header for logging

namespace ros_to_uav {

template <>
bool convert_request(const std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetNodeInfo::Request> ros_request, uavcan_protocol_GetNodeInfoRequest& uav_request)
{
    // Add a debug log to print the request data
    std::cout << "Converting ROS request to UAVCAN request..." << std::endl;
    // Add more log statements as needed to print the request data

    return true;
}

template <>
bool convert_response(const uavcan_protocol_GetNodeInfoResponse& uav_response, std::shared_ptr<uavcan_ros_msgs::srv::UavcanGetNodeInfo::Response> ros_response)
{
    // Add a debug log to print the response data
    std::cout << "Converting UAVCAN response to ROS response..." << std::endl;
    // Add more log statements as needed to print the response data

    // Determine the actual size of the name (up to 80 bytes)
    // size_t name_size = 0;
    // while (name_size < 80 && uav_response.name.data[name_size] != 0) {
    //     name_size++;
    // }

    // // Resize the ROS response name and copy the data
    // ros_response->name.resize(name_size);
    // for (size_t i = 0; i < name_size; ++i) {
    //     ros_response->name[i] = uav_response.name.data[i];
    // }

    // Copy other fields
    // ros_response->name = uav_response.name.data;
    ros_response->status.uptime_sec = uav_response.status.uptime_sec;
    ros_response->status.mode = uav_response.status.mode;
    ros_response->status.sub_mode = uav_response.status.sub_mode;
    ros_response->status.vendor_specific_status_code = uav_response.status.vendor_specific_status_code;

    // Print the ROS response in the terminal
    std::cout << "ROS response: " << std::endl;
    std::cout << "Name: " << ros_response->name << std::endl;
    std::cout << "Uptime (sec): " << ros_response->status.uptime_sec << std::endl;
    std::cout << "Mode: " << ros_response->status.mode << std::endl;
    std::cout << "Sub Mode: " << ros_response->status.sub_mode << std::endl;
    std::cout << "Vendor Specific Status Code: " << ros_response->status.vendor_specific_status_code << std::endl;

    return true;
}

}
