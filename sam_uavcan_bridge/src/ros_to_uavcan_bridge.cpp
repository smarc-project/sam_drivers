#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
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
// void initialize()
//     {
//         auto shared_this = shared_from_this();
//         dvl_server_ = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>>(
//             &canard_interface, shared_this, "dvl_command", 71);
//     }
private:
    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
    Canard::ObjCallback<RosToUavcanBridge, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{this, &RosToUavcanBridge::handle_GetNodeInfo};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{canard_interface, node_info_req_cb};

    
    void send_NodeStatus(void);
    rclcpp::TimerBase::SharedPtr timer_;
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
    // std::unique_ptr<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>> toggle_7v_server; 


};






static uint64_t micros64(void)
{
    static uint64_t first_us;                                                                                           
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t tus = (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
    if (first_us == 0) {
        first_us = tus;
    }
    return tus - first_us;
}

/*
  get monotonic time in milliseconds since startup
 */
static uint32_t millis32(void)
{
    return micros64() / 1000ULL;
}

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




void CanardInterface::init(const char *interface_name, uint8_t node_id) {
    int16_t res = socketcanInit(&socketcan, interface_name);
    if (res < 0) {
        (void)fprintf(stderr, "Failed to open CAN iface '%s'\n", interface_name);
        exit(1);
    }
        canardInit(&canard,memory_pool,sizeof(memory_pool),onTransferReceived, shouldAcceptTransfer,this);
        set_node_id(node_id);


} 

void CanardInterface::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    iface->handle_message(*transfer);
}

bool CanardInterface::shouldAcceptTransfer(const CanardInstance* ins,
                                   uint64_t* out_data_type_signature,
                                   uint16_t data_type_id,
                                   CanardTransferType transfer_type,
                                   uint8_t source_node_id)
{
    CanardInterface* iface = (CanardInterface*)ins->user_reference;
    return iface->accept_message(data_type_id, *out_data_type_signature);
}


/*
  Transmits all frames from the TX queue, receives up to one frame.
*/
void CanardInterface::process(uint32_t timeout_msec)
{
    // Transmitting
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        const int16_t tx_res = socketcanTransmit(&socketcan, txf, 0);
        if (tx_res != 0) {
            canardPopTxQueue(&canard);
        }
    }

    // Receiving
    const uint32_t start_ms = millis32();
    while (millis32() - start_ms < timeout_msec) {
        CanardCANFrame rx_frame;
        const int16_t rx_res = socketcanReceive(&socketcan, &rx_frame, timeout_msec);
        if (rx_res > 0) {
            canardHandleRxFrame(&canard, &rx_frame, micros64());
        }
    }
}
bool CanardInterface::broadcast(const Canard::Transfer &bcast_transfer) {
    tx_transfer.transfer_type = bcast_transfer.transfer_type;
    tx_transfer.data_type_signature = bcast_transfer.data_type_signature; 
    tx_transfer.data_type_id = bcast_transfer.data_type_id; 
    tx_transfer.inout_transfer_id = bcast_transfer.inout_transfer_id; 
    tx_transfer.priority = bcast_transfer.priority; 
    tx_transfer.payload = (const uint8_t*)bcast_transfer.payload; 
    tx_transfer.payload_len = static_cast<uint16_t>(bcast_transfer.payload_len); 
    // do canard broadcast
    bool success = canardBroadcastObj(&canard, &tx_transfer) > 0;
    return success;
}


bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) {
    tx_transfer.transfer_type = req_transfer.transfer_type; 
    tx_transfer.data_type_signature = req_transfer.data_type_signature; 
    tx_transfer.data_type_id = req_transfer.data_type_id; 
    tx_transfer.inout_transfer_id = req_transfer.inout_transfer_id; 
    tx_transfer.priority = req_transfer.priority; 
    tx_transfer.payload = (const uint8_t*)req_transfer.payload; 
    tx_transfer.payload_len = uint16_t(req_transfer.payload_len); 

    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}


bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) {
    tx_transfer.transfer_type = res_transfer.transfer_type; 
    tx_transfer.data_type_signature = res_transfer.data_type_signature; 
    tx_transfer.data_type_id = res_transfer.data_type_id; 
    tx_transfer.inout_transfer_id = res_transfer.inout_transfer_id; 
    tx_transfer.priority = res_transfer.priority; 
    tx_transfer.payload = (const uint8_t*)res_transfer.payload; 
    tx_transfer.payload_len = uint16_t(res_transfer.payload_len); 
    
    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}

void RosToUavcanBridge::start_canard_node() {
    start_node(can_interface_.c_str(), self_node_id_);
}
void RosToUavcanBridge::start_node(const char *can_interface_, u_int8_t node_id) {
    canard_interface.init(can_interface_, node_id);
    auto shared_this = shared_from_this();

    dvl_server_ = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool, ros_to_uav::DVLTag>>(
        &canard_interface, shared_this, "dvl_command", 71, ros_to_uav::DVLTag{});

    thrust_vector_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::ThrusterAngles>>(
        &canard_interface, shared_this, "vector_command", 16);

    tcg_server1 = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::BallastAngles>>(
        &canard_interface, shared_this, "tcg_command", 27);

    dropweight_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>>(
        &canard_interface, shared_this, "dropweight_command", 69);

    rpm1_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_esc_RPMCommand, smarc_msgs::msg::ThrusterRPM>>(
        &canard_interface, shared_this, "rpm1_command", 0);

    rpm2_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_esc_RPMCommand, smarc_msgs::msg::ThrusterRPM>>(
        &canard_interface, shared_this, "rpm2_command", 1);

    new_rpm1_server = std::make_unique<ros_to_uav::ConversionServer<smarc_uavcan_messages_ThrusterRpmID, smarc_msgs::msg::ThrusterRPM>>(
        &canard_interface, shared_this, "new_rpm1_command", 1);
    
    new_rpm2_server = std::make_unique<ros_to_uav::ConversionServer<smarc_uavcan_messages_ThrusterRpmID, smarc_msgs::msg::ThrusterRPM>>(
        &canard_interface, shared_this, "new_rpm2_command", 2);

    rpm_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_esc_RPMCommand, sam_msgs::msg::ThrusterRPMs>>(
        &canard_interface, shared_this, "rpm_command");

    vbs_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::PercentStamped>>(
        &canard_interface, shared_this, "vbs_command", 13);
    
    lcg_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::PercentStamped>>(
        &canard_interface, shared_this, "lcg_command", 14);

    array_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, sam_msgs::msg::ArrayCommand>>(
        &canard_interface, shared_this, "array_command");

    // tcg_server2 = std::make_unique<ros_to_uav::ConversionServer<uavcan::equipment::actuator::ArrayCommand, sam_msgs::BallastAngles>>(
    //     &canard_interface, shared_this, "tcg_command2", 28);

    dual_thruster_rpm_server = std::make_unique<ros_to_uav::ConversionServer<smarc_uavcan_messages_DualThrusterRPM, smarc_msgs::msg::DualThrusterRPM>>(
        &canard_interface, shared_this, "dual_thruster_rpm");

    light_command_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_indication_LightsCommand, sam_msgs::msg::LightCommand>>(
        &canard_interface, shared_this, "light_command");
    
    panic_forwardning_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_protocol_Panic, std_msgs::msg::String>>(
        &canard_interface, shared_this, "panic_forwardning_out");
    

    led_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>>(
        &canard_interface, shared_this, "led_command",70);
    
    sss_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>>(
        &canard_interface, shared_this, "sss_command", 72);

    // toggle_7v_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Bool>>(
    //     &canard_interface, shared_this, "toggle_7v_command", 7);
    command_server = std::make_unique<ros_to_uav::ConversionServer<uavcan_equipment_actuator_ArrayCommand, std_msgs::msg::Float32>>(
        &canard_interface, shared_this, "command");

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
    auto node = std::make_shared<RosToUavcanBridge>();
    node->start_canard_node();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}