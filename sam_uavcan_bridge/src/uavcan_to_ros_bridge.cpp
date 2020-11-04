#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <unordered_map>
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/uav_to_ros/imu.h>
#include <uavcan_ros_bridge/uav_to_ros/gps_fix.h>
#include <uavcan_ros_bridge/uav_to_ros/magnetic_field.h>
#include <uavcan_ros_bridge/uav_to_ros/pressure.h>
#include <uavcan_ros_bridge/uav_to_ros/sensor_pressure.h>
#include <uavcan_ros_bridge/uav_to_ros/esc_status.h>
#include <uavcan_ros_bridge/uav_to_ros/circuit_status.h>
#include <uavcan_ros_bridge/UavcanNodeStatusNamedArray.h>

#include <sam_uavcan_bridge/uav_to_ros/actuator_status.h>
#include <sam_uavcan_bridge/uav_to_ros/leak.h>
#include <sam_uavcan_bridge/uav_to_ros/sensor_pressure_stamped.h>
#include <sam_uavcan_bridge/uav_to_ros/servo_feedback_double.h>
#include <sam_uavcan_bridge/uav_to_ros/temperature.h>
#include <sam_uavcan_bridge/uav_to_ros/battery_state_basic.h>
#include <sam_uavcan_bridge/uav_to_ros/consumed_charge_feedback.h>
#include <sam_uavcan_bridge/uav_to_ros/ctd_feedback.h>
#include <sam_uavcan_bridge/uav_to_ros/dual_thruster_feedback.h>


class Monitor : public uavcan::INodeInfoListener
{
public:
    std::unordered_map<int, std::pair<std::string, uavcan::protocol::NodeStatus>> node_registry;

private:
    // std::unique_ptr<uavcan::protocol::GetNodeInfo::Response> last_node_info;
    // uavcan::NodeID last_node_id;
    // unsigned status_message_cnt;
    // unsigned status_change_cnt;
    // unsigned info_unavailable_cnt;

    virtual void handleNodeInfoRetrieved(uavcan::NodeID node_id,
                                         const uavcan::protocol::GetNodeInfo::Response& node_info)
    {
        std::string _name = "";
        for(auto c:node_info.name){
            _name += c;
        }
        node_registry[node_id.get()].first = _name;
        // last_node_info.reset(new uavcan::protocol::GetNodeInfo::Response(node_info));
    }

    #pragma GCC diagnostic ignored "-Wunused-parameter"
    virtual void handleNodeInfoUnavailable(uavcan::NodeID node_id)
    {
        // std::cout << "NODE INFO FOR " << int(node_id.get()) << " IS UNAVAILABLE" << std::endl;
        // last_node_id = node_id;
        // info_unavailable_cnt++;
    }
    #pragma GCC diagnostic pop

    #pragma GCC diagnostic ignored "-Wunused-parameter"
    virtual void handleNodeStatusChange(const uavcan::NodeStatusMonitor::NodeStatusChangeEvent& event)
    {
        // std::cout << "NODE " << int(event.node_id.get()) << " STATUS CHANGE: "
        //           << event.old_status.toString() << " --> " << event.status.toString() << std::endl;
        // status_change_cnt++;
    }
    #pragma GCC diagnostic pop

    virtual void handleNodeStatusMessage(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
    {
        node_registry[msg.getSrcNodeID().get()].second = msg;
        // status_message_cnt++;
    }
public:
    Monitor()
        // : status_message_cnt(0)
        // , status_change_cnt(0)
        // , info_unavailable_cnt(0)
    { }
};

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

    uav_to_ros::ConversionServer<uavcan::equipment::device::Temperature, sensor_msgs::Temperature> motor_oil_temperature_server(uav_node, pn, "motor_temperature", 1);
    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::PercentStamped> vbs_feedback_server(uav_node, pn, "vbs_feedback", 13);
    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::PercentStamped> lcg_feedback_server(uav_node, pn, "lcg_feedback", 14);

    uav_to_ros::ConversionServer<uavcan::equipment::actuator::Status, sam_msgs::Leak> leak_server(uav_node, pn, "leak", 200);
    uav_to_ros::ConversionServer<uavcan::equipment::esc::Status, uavcan_ros_bridge::ESCStatus> esc_status_server0(uav_node, pn, "esc_status0", 0);
    uav_to_ros::ConversionServer<uavcan::equipment::esc::Status, uavcan_ros_bridge::ESCStatus> esc_status_server1(uav_node, pn, "esc_status1", 1);
    uav_to_ros::ConversionServer<uavcan::equipment::power::CircuitStatus, uavcan_ros_bridge::CircuitStatus> circuit_status_server(uav_node, pn, "circuit_status");
    uav_to_ros::ConversionServer<smarc_uavcan_messages::BatteryStateBasic, sensor_msgs::BatteryState> battery_server2(uav_node, pn, "battery_state_basic");
    uav_to_ros::ConversionServer<smarc_uavcan_messages::ConsumedChargeFeedback, sam_msgs::ConsumedChargeFeedback> consumed_charge_server(uav_node, pn, "consumed_charge_feedback");
    uav_to_ros::ConversionServer<smarc_uavcan_messages::CTDFeedback, smarc_msgs::CTDFeedback> ctd_feedback_server(uav_node, pn, "ctd_feedback");
    uav_to_ros::ConversionServer<smarc_uavcan_messages::DualThrusterFeedback, smarc_msgs::DualThrusterFeedback> thruster_feedback_server(uav_node, pn, "thrusters_feedback");

    uavcan::NodeInfoRetriever monitor(uav_node);
    const int monitor_init_res = monitor.start();
    if (monitor_init_res < 0) {
        ROS_ERROR("Failed to start the node status monitor; error: %d", monitor_init_res);
        exit(0);
    }
    else
    {
        ROS_INFO("UAVCAN network monitor started");
    }

    Monitor listener;
    monitor.addListener(&listener);

    ros::Publisher status_pub = pn.advertise<uavcan_ros_bridge::UavcanNodeStatusNamedArray>("uavcan_network_status", 1);
    uavcan_ros_bridge::UavcanNodeStatusNamedArray msgArray;

    uavcan::Timer monitor_timer(uav_node);
    monitor_timer.setCallback([&](const uavcan::TimerEvent&) {
        for (unsigned i = 1; i <= uavcan::NodeID::Max; i++)
        {
            if (monitor.isNodeKnown(i))
            {
                // ROS_INFO("Found ID %d", i);
                // ROS_INFO("Name: %s", listener.node_registry[i].first.c_str());
                uavcan_ros_bridge::UavcanNodeStatusNamed msgNode;
                msgNode.id = i;
                msgNode.name = listener.node_registry[i].first;
                msgNode.ns.uptime_sec = listener.node_registry[i].second.uptime_sec;
                msgNode.ns.health = listener.node_registry[i].second.health;
                msgNode.ns.mode = listener.node_registry[i].second.mode;
                msgNode.ns.sub_mode = listener.node_registry[i].second.mode;
                msgNode.ns.vendor_specific_status_code = listener.node_registry[i].second.sub_mode;
                msgArray.array.push_back(msgNode);
            }
        }
        status_pub.publish(msgArray);
    });

    monitor_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000));

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
