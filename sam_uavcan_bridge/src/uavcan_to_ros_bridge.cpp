#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/uav_to_ros/imu.h>
#include <uavcan_ros_bridge/uav_to_ros/gps_fix.h>
#include <uavcan_ros_bridge/uav_to_ros/magnetic_field.h>
#include <uavcan_ros_bridge/uav_to_ros/pressure.h>
#include <uavcan_ros_bridge/uav_to_ros/sensor_pressure.h>
#include <uavcan_ros_bridge/uav_to_ros/esc_status.h>
// #include <uavcan_ros_bridge/uav_to_ros/circuit_status.h>

#include <sam_uavcan_bridge/uav_to_ros/actuator_status.h>
#include <sam_uavcan_bridge/uav_to_ros/leak.h>
#include <sam_uavcan_bridge/uav_to_ros/sensor_pressure_stamped.h>
#include <sam_uavcan_bridge/uav_to_ros/servo_feedback_double.h>
#include <sam_uavcan_bridge/uav_to_ros/temperature.h>
#include <sam_uavcan_bridge/uav_to_ros/battery_state_basic.h>
// #include <sam_uavcan_bridge/uav_to_ros/consumed_charge_feedback.h>
#include <sam_uavcan_bridge/uav_to_ros/consumed_charge_array.h>
#include <sam_uavcan_bridge/uav_to_ros/ctd_feedback.h>
#include <sam_uavcan_bridge/uav_to_ros/thruster_feedback_id.h>
#include <sam_uavcan_bridge/uav_to_ros/circuit_status_stamped.h>
#include <sam_uavcan_bridge/uav_to_ros/panic.h>

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
    uav_to_ros::ConversionServer<uavcan::equipment::ahrs::MagneticFieldStrength, sensor_msgs::MagneticField> magnetic_server(uav_node, pn, "magnetic_field");

    uav_to_ros::ConversionServer<smarc_uavcan_messages::SensorPressureStamped, sensor_msgs::FluidPressure> sensor_pressure_bar30(uav_node, pn, "sensor_pressure_bar30", 22);
    uav_to_ros::ConversionServer<smarc_uavcan_messages::SensorPressureStamped, sensor_msgs::FluidPressure> sensor_pressure_bar02(uav_node, pn, "sensor_pressure_bar02", 23);
    uav_to_ros::ConversionServer<smarc_uavcan_messages::SensorPressureStamped, sensor_msgs::FluidPressure> motor_oil_pressure_server(uav_node, pn, "motor_oil_pressure", 1);
    uav_to_ros::ConversionServer<smarc_uavcan_messages::SensorPressureStamped, sensor_msgs::FluidPressure> vbs_tank_pressure_server(uav_node, pn, "vbs_tank_pressure", 2);
    uav_to_ros::ConversionServer<uavcan::equipment::device::Temperature, sensor_msgs::Temperature> vbs_tank_temperature_server(uav_node, pn, "vbs_tank_temperature", 2);

    uav_to_ros::ConversionServer<uavcan::equipment::device::Temperature, sensor_msgs::Temperature> motor_oil_temperature_server(uav_node, pn, "motor_temperature", 1);
    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::PercentStamped> vbs_feedback_server(uav_node, pn, "vbs_feedback", 13);
    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::PercentStamped> lcg_feedback_server(uav_node, pn, "lcg_feedback", 14);

    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::Leak> leak_server(uav_node, pn, "leak", 200);
    // uav_to_ros::ConversionServer<uavcan::equipment::esc::Status, uavcan_ros_msgs::ESCStatus> esc_status_server0(uav_node, pn, "esc_status0", 0);
    // uav_to_ros::ConversionServer<uavcan::equipment::esc::Status, uavcan_ros_msgs::ESCStatus> esc_status_server1(uav_node, pn, "esc_status1", 1);
    uav_to_ros::ConversionServer<uavcan::equipment::power::CircuitStatus, sam_msgs::CircuitStatusStamped> circuit_status_server(uav_node, pn, "circuit_status");
    uav_to_ros::ConversionServer<smarc_uavcan_messages::BatteryStateBasic, sensor_msgs::BatteryState> battery_server2(uav_node, pn, "battery_state_basic");
    uav_to_ros::ConversionServer<smarc_uavcan_messages::ConsumedChargeArray, sam_msgs::ConsumedChargeArray> consumed_charge_server2(uav_node, pn, "consumed_charge_array");
    uav_to_ros::ConversionServer<smarc_uavcan_messages::CTDFeedback, smarc_msgs::CTD> ctd_feedback_server(uav_node, pn, "ctd_feedback");
    uav_to_ros::ConversionServer<smarc_uavcan_messages::ThrusterFeedbackID, smarc_msgs::ThrusterFeedback> thruster1_feedback_server(uav_node, pn, "thruster1_feedback", 1);
    uav_to_ros::ConversionServer<smarc_uavcan_messages::ThrusterFeedbackID, smarc_msgs::ThrusterFeedback> thruster2_feedback_server(uav_node, pn, "thruster2_feedback", 2);
    uav_to_ros::ConversionServer<uavcan::protocol::Panic, std_msgs::String> panic_forwarding_server(uav_node, pn, "panic_forwarding_in");

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
