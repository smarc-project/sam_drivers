#include <sam_uavcan_bridge/ros_to_uav/uavcan_update_battery.h>


namespace ros_to_uav {

template <>
bool convert_request(const sam_msgs::UavcanUpdateBattery::Request& ros_request, smarc_uavcan_services::UpdateBattery::Request& uav_request)
{
    uav_request.command = ros_request.command;
    uav_request.charge = ros_request.charge;
    return true;
}

template <>
bool convert_response(const smarc_uavcan_services::UpdateBattery::Response& uav_response, sam_msgs::UavcanUpdateBattery::Response& ros_response)
{
    ros_response.success = uav_response.success;
    return true;
}

}
