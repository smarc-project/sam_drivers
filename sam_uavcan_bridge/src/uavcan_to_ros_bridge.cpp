#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/uav_to_ros/imu.h>
#include <uavcan_ros_bridge/uav_to_ros/gps_fix.h>
#include <uavcan_ros_bridge/uav_to_ros/battery_state.h>
#include <uavcan_ros_bridge/uav_to_ros/magnetic_field.h>
#include <uavcan_ros_bridge/uav_to_ros/pressure.h>
#include <uavcan_ros_bridge/uav_to_ros/sensor_pressure.h>

#include <sam_uavcan_bridge/uav_to_ros/actuator_status.h>
#include <sam_uavcan_bridge/uav_to_ros/leak.h>

extern uavcan::ICanDriver& getCanDriver(const std::string&);
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

static Node& getNode(const std::string& can_interface)
{
    static Node node(getCanDriver(can_interface), getSystemClock());
    return node;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uavcan_to_ros_bridge_node");
    ros::NodeHandle ros_node;

    int self_node_id;
    std::string can_interface;
    ros::param::param<int>("~uav_node_id", self_node_id, 114);
    ros::param::param<std::string>("~uav_can_interface", can_interface, "can0");

    auto& uav_node = getNode(can_interface);
    uav_node.setNodeID(self_node_id);
    uav_node.setName("smarc.sam.uavcan_bridge.subscriber");

     /*
     * Configuring the Data Type IDs.
     * See the server sources for details.
     */
    /*
    auto regist_result = uavcan::GlobalDataTypeRegistry::instance().registerDataType<smarc_uavcan_messages::SensorPressure>(243);
    if (regist_result != uavcan::GlobalDataTypeRegistry::RegistrationResultOk) {
        ROS_ERROR("Failed to register the data type: %d", regist_result);
        exit(0);
    }
    */

    /*
     * Dependent objects (e.g. publishers, subscribers, servers, callers, timers, ...) can be initialized only
     * if the node is running. Note that all dependent objects always keep a reference to the node object.
     */
    const int node_start_res = uav_node.start();
    if (node_start_res < 0) {
        ROS_ERROR("Failed to start the node; error: %d", node_start_res);
        exit(0);
    }

    ros::NodeHandle pn("~");
    uav_to_ros::ConversionServer<uavcan::equipment::ahrs::Solution, sensor_msgs::Imu> imu_server(uav_node, pn, "imu");
    uav_to_ros::ConversionServer<uavcan::equipment::gnss::Fix, sensor_msgs::NavSatFix> gps_server(uav_node, pn, "gps_fix");
    uav_to_ros::ConversionServer<uavcan::equipment::power::BatteryInfo, sensor_msgs::BatteryState> battery_server(uav_node, pn, "battery_state");
    uav_to_ros::ConversionServer<uavcan::equipment::ahrs::MagneticFieldStrength, sensor_msgs::MagneticField> magnetic_server(uav_node, pn, "magnetic_field");
    // NOTE: the last argument is the source node id numbers, these are example values
    uav_to_ros::ConversionServer<uavcan::equipment::air_data::StaticPressure, sensor_msgs::FluidPressure> pressure_server1(uav_node, pn, "pressure1", 91);
    //uav_to_ros::ConversionServer<uavcan::equipment::air_data::StaticPressure, sensor_msgs::FluidPressure> pressure_server2(uav_node, pn, "pressure2", 93);

    uav_to_ros::ConversionServer<smarc_uavcan_messages::SensorPressure, sensor_msgs::FluidPressure> sensor_pressure_server1(uav_node, pn, "sensor_pressure1", 22);
    uav_to_ros::ConversionServer<smarc_uavcan_messages::SensorPressure, sensor_msgs::FluidPressure> sensor_pressure_server2(uav_node, pn, "shallow_pressure", 23);
    //uav_to_ros::ConversionServer<smarc_uavcan_messages::SensorPressure, sensor_msgs::FluidPressure> motor_oil_pressure_server(uav_node, pn, "motor_oil_pressure", 20);
    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::PercentStamped> vbs_feedback_server(uav_node, pn, "vbs_feedback", 13);
    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::PercentStamped> lcg_feedback_server(uav_node, pn, "lcg_feedback", 14);

    // TODO: replace these with a proper uavcan message with both angles included
    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::PercentStamped> tcg_feedback_server1(uav_node, pn, "tcg_feedback1", 27);
    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::PercentStamped> tcg_feedback_server2(uav_node, pn, "tcg_feedback2", 28);

    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::Leak> leak_server(uav_node, pn, "leak", 200);

    /*
     * Running the node.
     */
    uav_node.setModeOperational();
    signal(SIGINT, [] (int) { ros::shutdown(); });

    while (ros::ok()) {
        /*
         * The method spin() may return earlier if an error occurs (e.g. driver failure).
         * All error codes are listed in the header uavcan/error.hpp.
         */
        const int res = uav_node.spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0) {
            ROS_ERROR("Transient failure or shutdown: %d", res);
        }
    }

    return 0;
}
