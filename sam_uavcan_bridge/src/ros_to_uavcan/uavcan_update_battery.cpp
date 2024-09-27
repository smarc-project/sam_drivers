#include <uavcan_update_battery.h>


namespace ros_to_uav {

template <>
bool convert_request(const std::shared_ptr<sam_msgs::srv::UavcanUpdateBattery::Request> ros_request, smarc_uavcan_services_UpdateBatteryRequest& uav_request)
{
    uav_request.command = ros_request->command;
    uav_request.charge = ros_request->charge;
    return true;
}

template <>
bool convert_response(const smarc_uavcan_services_UpdateBatteryResponse& uav_response, std::shared_ptr<sam_msgs::srv::UavcanUpdateBattery::Response> ros_response)
{
    ros_response->success = uav_response.success;
    return true;
}

}
