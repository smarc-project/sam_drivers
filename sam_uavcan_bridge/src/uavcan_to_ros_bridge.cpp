#include <uavcan_ros_bridge.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <uavcan_to_ros/pressure.h>
#include <uavcan_to_ros/actuator_status.h>
#include <uavcan_to_ros/panic.h>
#include <time_utils.h>
#include <sam_msgs/msg/percent_stamped.hpp>

DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS(); 

class UavcanToRosBridge : public rclcpp::Node
{
    public:
    UavcanToRosBridge() : Node("uavcan_to_ros_bridge")
    {
        self_node_id_= this ->declare_parameter("uav_node_id",117);
        can_interface_ = this -> declare_parameter("uav_can_interface","vcan0"); 
    }
 
    void start_node(const char *can_interface_, uint8_t node_id);
    void start_canard_node();
private:

    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
    Canard::ObjCallback<UavcanToRosBridge, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{this, &UavcanToRosBridge::handle_GetNodeInfo};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{canard_interface, node_info_req_cb};

    
    void send_NodeStatus(void);
    rclcpp::TimerBase::SharedPtr timer_;
    uavcan_protocol_NodeStatus msg;
    int self_node_id_;
    std::string can_interface_;
    CanardInterface canard_interface{0};   
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub{canard_interface}; 
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_air_data_StaticPressure,sensor_msgs::msg::FluidPressure>> sensore_pressure_server1;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_actuator_Status,sam_msgs::msg::PercentStamped>> lcg_feedback_server;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_protocol_Panic,std_msgs::msg::String>> panic_forwarding_server;



};


//Functions to send NodeStatus to the Dronecan Gui
void UavcanToRosBridge::send_NodeStatus(void)
 {
    msg.uptime_sec = millis32() / 1000UL;
    msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    msg.sub_mode = 0;
    
    node_status_pub.broadcast(msg);
}


static void getUniqueID(uint8_t id[16])
{
    memset(id, 0, 16);
    FILE *f = fopen("/etc/machine-id", "r");
    if (f) {
        fread(id, 1, 16, f);
        fclose(f);
    }
}
void UavcanToRosBridge::handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req) {
    uavcan_protocol_GetNodeInfoResponse response{};
    
    response.name.len = snprintf((char*)response.name.data, sizeof(response.name.data), "UavcanToRosBridge");
    response.software_version.major = 1;
    response.software_version.minor = 2;
    response.hardware_version.major = 3;
    response.hardware_version.minor = 7;
    getUniqueID(response.hardware_version.unique_id);
    response.status = msg;
    response.status.uptime_sec = millis32() / 1000UL;
    node_info_server.respond(transfer, response);
    
}


void UavcanToRosBridge::start_canard_node() {
    start_node(can_interface_.c_str(), self_node_id_);
}
void UavcanToRosBridge::start_node(const char *can_interface_, u_int8_t node_id) {
    canard_interface.init(can_interface_, node_id);
    auto shared_this = shared_from_this();

    sensore_pressure_server1 = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_air_data_StaticPressure,sensor_msgs::msg::FluidPressure>>(
    &canard_interface , shared_this, "sensore_pressure1",1);

    lcg_feedback_server = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_actuator_Status,sam_msgs::msg::PercentStamped>>(
    &canard_interface , shared_this, "lcg_feedback",14);

    panic_forwarding_server = std::make_unique<uav_to_ros::ConversionServer<uavcan_protocol_Panic,std_msgs::msg::String>>(
    &canard_interface , shared_this, "panic_forwarding_in");




    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // Adjust the period as needed
        [this]() {
            canard_interface.process(1);
            const uint64_t ts = micros64();
            static uint64_t next_50hz_service_at = ts;
            if (ts >= next_50hz_service_at) {
                next_50hz_service_at += 1000000ULL / 50U;
                this->send_NodeStatus();
            }
        }
    );
    
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UavcanToRosBridge>();
    node->start_canard_node();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}