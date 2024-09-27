#include <uavcan_to_ros/gps_fix.h>

namespace uav_to_ros {

template <>
bool convert(const uavcan_equipment_gnss_Fix& uav_msg, std::shared_ptr<sensor_msgs::msg::NavSatFix> ros_msg)
{
    ros_msg->header.stamp = convert_timestamp(uav_msg.timestamp.usec);
    ros_msg->latitude = 1e-8*double(uav_msg.latitude_deg_1e8);
    ros_msg->longitude = 1e-8*double(uav_msg.longitude_deg_1e8);
    ros_msg->altitude = 1e-3*double(uav_msg.height_ellipsoid_mm);

    if (uav_msg.status == UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_NO_FIX) {
        ros_msg->status.status = ros_msg->status.STATUS_NO_FIX;
    }
    else {
        ros_msg->status.status = ros_msg->status.STATUS_FIX;
    }

    for (int i = 0; i < 9; ++i) {
        ros_msg->position_covariance[i] = uav_msg.position_covariance.data[i];
    }

    return true;
}

}
