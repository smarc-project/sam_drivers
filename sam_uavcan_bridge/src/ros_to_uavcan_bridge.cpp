#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <time_utils.h>
#include <uavcan_ros_bridge.h>
#include <dvl.h>
#include <thruster_rpm.h>
#include <thruster_rpm_id.h>
#include <ballast_angles.h>
#include <thruster_angles.h>
#include <thruster_rpms.h>
#include <dropweights.h>
#include <percent_stamped.h>
#include <light_command.h>
#include <panic.h>
// #include <toggle_7v.h>
#include <command.h>
#include <dual_thruster_rpm.h>
#include <led.h>
#include <light_command.h>
#include <sss.h>
#include <array_command.h>
#include <sam_msgs/msg/topics.hpp>

#define MY_NODE_ID 113


DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS();
class RosToUavcanBridge : public rclcpp::Node
{
public:
   RosToUavcanBridge() : Node("ros_to_uavcan_bridge_node")
{
    self_node_id_ = this->declare_parameter<int>("uav_node_id", MY_NODE_ID);
    can_interface_ = this->declare_parameter<std::string>("uav_can_interface", "vcan0");
}

    void start_node(const char *can_interface_, uint8_t node_id);
    void start_canard_node();
private:

    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
    Canard::ObjCallback<RosToUavcanBridge, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{this, &RosToUavcanBridge::handle_GetNodeInfo};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{canard_interface, node_info_req_cb};

    
    void send_NodeStatus(void);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_1hz;
    uavcan_protocol_NodeStatus msg;
    int self_node_id_;
    std::string can_interface_;
    CanardInterface canard_interface{0};    
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub{canard_interface};

    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool, ros_to_uav::DVLTag>> dvl_server_;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::ThrusterAngles>> thrust_vector_server;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::BallastAngles>> tcg_server1;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>> dropweight_server;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Float32>> command_server;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_esc_RPMCommand, smarc_msgs::msg::ThrusterRPM>> rpm1_server;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_esc_RPMCommand, smarc_msgs::msg::ThrusterRPM>> rpm2_server;
    std::unique_ptr<ros_to_uav::ConversionServer<smarc_uavcan_messages_ThrusterRpmID, smarc_msgs::msg::ThrusterRPM>> new_rpm1_server;
    std::unique_ptr<ros_to_uav::ConversionServer<smarc_uavcan_messages_ThrusterRpmID, smarc_msgs::msg::ThrusterRPM>> new_rpm2_server;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_esc_RPMCommand, sam_msgs::msg::ThrusterRPMs>> rpm_server;

    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::ArrayCommand>> array_server ;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::PercentStamped>> vbs_server ;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::PercentStamped>> lcg_server;
    // std::unique_ptr<ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::BallastAngles>> tcg_server2;
    std::unique_ptr<ros_to_uav::ConversionServer<smarc_uavcan_messages_DualThrusterRPM, smarc_msgs::msg::DualThrusterRPM>> dual_thruster_rpm_server;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_indication_LightsCommand, sam_msgs::msg::LightCommand>> light_command_server ;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_protocol_Panic, std_msgs::msg::String>> panic_forwardning_server;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>> led_server; 
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>> sss_server ;
    std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>> toggle_7v_server; 


};
//Function to send NodeStatus to the Dronecan Gui
void RosToUavcanBridge::send_NodeStatus(void)
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
void RosToUavcanBridge::handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req) {
    uavcan_protocol_GetNodeInfoResponse response{};
    
    response.name.len = snprintf((char*)response.name.data, sizeof(response.name.data), "RosToUavcanBridge");
    response.software_version.major = 1;
    response.software_version.minor = 2;
    response.hardware_version.major = 3;
    response.hardware_version.minor = 7;
    getUniqueID(response.hardware_version.unique_id);
    response.status = msg;
    response.status.uptime_sec = millis32() / 1000UL;
    node_info_server.respond(transfer, response);
    
}



void RosToUavcanBridge::start_canard_node() {
    start_node(can_interface_.c_str(), self_node_id_);
}
void RosToUavcanBridge::start_node(const char *can_interface_, u_int8_t node_id) {
    canard_interface.init(can_interface_, node_id);
    auto shared_this = shared_from_this();

    dvl_server_ = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool, ros_to_uav::DVLTag>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::DVL_CMD_TOPIC, 71, ros_to_uav::DVLTag{});

    thrust_vector_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::ThrusterAngles>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::THRUST_VECTOR_CMD_TOPIC, 16);

    tcg_server1 = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::BallastAngles>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::TCG_CMD_TOPIC, 27);

    dropweight_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::DROPWEIGHTS_CMD_TOPIC, 69);

    rpm1_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_esc_RPMCommand, smarc_msgs::msg::ThrusterRPM>>(
        &canard_interface, shared_this, "rpm1_command", 0);

    rpm2_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_esc_RPMCommand, smarc_msgs::msg::ThrusterRPM>>(
        &canard_interface, shared_this, "rpm2_command", 1);

    new_rpm1_server = std::make_unique<ros_to_uav::ConversionServer<smarc_uavcan_messages_ThrusterRpmID, smarc_msgs::msg::ThrusterRPM>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::THRUSTER1_CMD_TOPIC, 1);
    
    new_rpm2_server = std::make_unique<ros_to_uav::ConversionServer<smarc_uavcan_messages_ThrusterRpmID, smarc_msgs::msg::ThrusterRPM>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::THRUSTER2_CMD_TOPIC, 2);

    rpm_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_esc_RPMCommand, sam_msgs::msg::ThrusterRPMs>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::RPM_CMD_TOPIC, 3);

    vbs_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::PercentStamped>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::VBS_CMD_TOPIC, 13);
    
    lcg_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::PercentStamped>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::LCG_CMD_TOPIC, 14);

    array_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::ArrayCommand>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::UTIL_CMD_TOPIC, 15);

    // tcg_server2 = std::make_unique<ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::BallastAngles>>(
    //     &canard_interface, shared_this, "tcg_command2", 28);

    dual_thruster_rpm_server = std::make_unique<ros_to_uav::ConversionServer<smarc_uavcan_messages_DualThrusterRPM, smarc_msgs::msg::DualThrusterRPM>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::DUAL_THRUSTER_CMD_TOPIC, 29);

    light_command_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_indication_LightsCommand, sam_msgs::msg::LightCommand>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::LIGHTS_CMD_TOPIC, 30);
    
    panic_forwardning_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_protocol_Panic, std_msgs::msg::String>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::PANIC_CMD_TOPIC, 31);
    

    led_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::LED_CMD_TOPIC,70);
    
    sss_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::SSS_CMD_TOPIC, 72);

    toggle_7v_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>>(
        &canard_interface, shared_this, sam_msgs::msg::Topics::TOGGLE_7V_CMD_TOPIC, 7);


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // Adjust the period as needed
        [this]() {
            canard_interface.process(1);
            // const uint64_t ts = micros64();
            // static uint64_t next_1hz_service_at = ts;
            // if (ts >= next_1hz_service_at) {
            //     next_1hz_service_at += 1000000ULL ;
            //     this->send_NodeStatus();
            // }
        }
    );
    timer_1hz = this->create_wall_timer(
        std::chrono::milliseconds(1000), // Adjust the period as needed
        [this]() {
            this->send_NodeStatus();
        }
    );
    
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosToUavcanBridge>();
    node->start_canard_node();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}