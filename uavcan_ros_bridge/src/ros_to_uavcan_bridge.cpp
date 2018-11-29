#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <uavcan_ros_bridge/uavcan_ros_bridge.h>
#include <uavcan_ros_bridge/ros_to_uav/command.h>
#include <uavcan_ros_bridge/ros_to_uav/rpm_command.h>
#include <uavcan_ros_bridge/ros_to_uav/array_command.h>
#include <uavcan_ros_bridge/ros_to_uav/percent_stamped.h>
#include <uavcan_ros_bridge/ros_to_uav/ballast_angles.h>

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_to_uavcan_bridge_node");
    ros::NodeHandle ros_node;

    int self_node_id;
    ros::param::param<int>("~uav_node_id", self_node_id, 113);
    
    auto& uav_node = getNode();
    uav_node.setNodeID(self_node_id);
    uav_node.setName("smarc.sam.uavcan_bridge.publisher");

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
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, std_msgs::Float32> command_server(uav_node, pn, "command");
    ros_to_uav::ConversionServer<uavcan::equipment::esc::RPMCommand, std_msgs::Int32> rpm_server(uav_node, pn, "rpm_command");
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::ArrayCommand> array_server(uav_node, pn, "array_command");
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::PercentStamped> vbs_server(uav_node, pn, "vbs_command", 13);
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::PercentStamped> tcg_server(uav_node, pn, "tcg_command", 14);
    ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::BallastAngles> lcg_server(uav_node, pn, "lcg_command", 15);

    /*
     * Running the node.
     */
    uav_node.setModeOperational();

    std::function<void (const ros::TimerEvent&)> callback = [&] (const ros::TimerEvent&) {
        // Announce that the uav node is alive and well
        uav_node.spinOnce();
    };
    ros::Timer timer = ros_node.createTimer(ros::Duration(0.5), callback);

    ros::spin();

    return 0;
}
