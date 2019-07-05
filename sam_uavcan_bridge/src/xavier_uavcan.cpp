#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/ros_to_uav/rpm_command.h>
#include <sam_uavcan_bridge/ros_to_uav/thruster_rpms.h>

#include <uavcan_ros_bridge/uav_to_ros/esc_status.h>

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
    ros::init(argc, argv, "ros_xavier_uavcan_node");
    ros::NodeHandle ros_node;

    int self_node_id;
    std::string can_interface;
    ros::param::param<int>("~uav_node_id", self_node_id, 113);
    ros::param::param<std::string>("~uav_can_interface", can_interface, "can0");
    
    auto& uav_node = getNode(can_interface);
    uav_node.setNodeID(self_node_id);
    uav_node.setName("smarc.sam.uavcan_bridge.xavier");

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
    ros_to_uav::ConversionServer<uavcan::equipment::esc::RPMCommand, sam_msgs::ThrusterRPMs> rpm_server(uav_node, pn, "rpm_command");

    uav_to_ros::ConversionServer<uavcan::equipment::esc::Status, uavcan_ros_bridge::ESCStatus> esc_status_server(uav_node, pn, "esc_status");
    
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

    // std::function<void (const ros::TimerEvent&)> callback = [&] (const ros::TimerEvent&) {
    //     // Announce that the uav node is alive and well
    //     uav_node.spinOnce();
    // };
    // ros::Timer timer = ros_node.createTimer(ros::Duration(0.5), callback);

    // ros::spin();

    return 0;
}
