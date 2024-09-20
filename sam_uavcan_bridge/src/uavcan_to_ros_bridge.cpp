#include <uavcan_ros_bridge.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <uavcan_to_ros/esc_status.h>
#include <uavcan_to_ros/gps_fix.h>
#include <uavcan_to_ros/magnetic_field.h>
#include <uavcan_to_ros/imu.h>
#include <uavcan_to_ros/sensor_pressure.h>

#include <uavcan_to_ros/pressure.h>
#include <uavcan_to_ros/actuator_status.h>
#include <uavcan_to_ros/panic.h>
#include <uavcan_to_ros/battery_info.h>
#include <uavcan_to_ros/battery_state_basic.h>
#include <uavcan_to_ros/circuit_status_stamped.h>
#include <uavcan_to_ros/consumed_charge_array.h>
#include <uavcan_to_ros/consumed_charge_feedback.h>
#include <uavcan_to_ros/ctd_feedback.h>
#include <uavcan_to_ros/dual_thruster_feedback.h>
#include <uavcan_to_ros/leak.h>
#include <uavcan_to_ros/sensor_pressure_stamped.h>
#include <uavcan_to_ros/servo_feedback_double.h>
#include <uavcan_to_ros/temperature.h>
#include <uavcan_to_ros/thruster_feedback_id.h>
#include <time_utils.h>
#include <sam_msgs/msg/percent_stamped.hpp>
#include <sam_msgs/msg/topics.hpp>

DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS(); 

class UavcanToRosBridge : public rclcpp::Node
{
public:
    UavcanToRosBridge() : Node("uavcan_to_ros_bridge")
    {
        self_node_id_ = this->declare_parameter("uav_node_id", 117);
        can_interface_ = this->declare_parameter("uav_can_interface", "vcan0");
    }

    void start_node(const char *can_interface_, uint8_t node_id);
    void start_canard_node();

private:
    CanardInterface canard_interface{0};
    void handle_GetNodeInfo(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest &req);
    Canard::ObjCallback<UavcanToRosBridge, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{this, &UavcanToRosBridge::handle_GetNodeInfo};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{canard_interface, node_info_req_cb};

    void send_NodeStatus(void);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_1hz;
    uavcan_protocol_NodeStatus msg;
    int self_node_id_;
    std::string can_interface_;
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub{canard_interface};
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_ahrs_Solution, sensor_msgs::msg::Imu>> imu_server;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_gnss_Fix, sensor_msgs::msg::NavSatFix>> gps_fix;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_protocol_Panic, std_msgs::msg::String>> panic_forwarding_server;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_ahrs_MagneticFieldStrength, sensor_msgs::msg::MagneticField>> magnetic_field;
    std::unique_ptr<uav_to_ros::ConversionServer<smarc_uavcan_messages_SensorPressureStamped, sensor_msgs::msg::FluidPressure>> sensor_pressure_bar30;
    std::unique_ptr<uav_to_ros::ConversionServer<smarc_uavcan_messages_SensorPressureStamped, sensor_msgs::msg::FluidPressure>> sensor_pressure_bar02;
    std::unique_ptr<uav_to_ros::ConversionServer<smarc_uavcan_messages_SensorPressureStamped, sensor_msgs::msg::FluidPressure>> motor_oil_pressure;
    std::unique_ptr<uav_to_ros::ConversionServer<smarc_uavcan_messages_SensorPressureStamped, sensor_msgs::msg::FluidPressure>> vbs_tank_pressure;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_device_Temperature, sensor_msgs::msg::Temperature>> vbs_tank_temperature;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_device_Temperature, sensor_msgs::msg::Temperature>> motor_oil_temperature;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_actuator_Status, sam_msgs::msg::PercentStamped>> vbs_feedback;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_actuator_Status, sam_msgs::msg::PercentStamped>> lcg_feedback;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_actuator_Status, sam_msgs::msg::Leak>> leak;
    // std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_esc_Status, uavcan_ros_msgs::ESCStatus>> esc_status_server0;
    // std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_esc_Status, uavcan_ros_msgs::ESCStatus>> esc_status_server1;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_power_CircuitStatus, sam_msgs::msg::CircuitStatusStamped>> circuit_status;
    std::unique_ptr<uav_to_ros::ConversionServer<smarc_uavcan_messages_BatteryStateBasic, sensor_msgs::msg::BatteryState>> battery_server2;
    std::unique_ptr<uav_to_ros::ConversionServer<uavcan_equipment_power_BatteryInfo, sensor_msgs::msg::BatteryState>> battery_server3;
    std::unique_ptr<uav_to_ros::ConversionServer<smarc_uavcan_messages_ConsumedChargeArray, sam_msgs::msg::ConsumedChargeArray>> consumed_charge_array;
    std::unique_ptr<uav_to_ros::ConversionServer<smarc_uavcan_messages_CTDFeedback, smarc_msgs::msg::CTD>> ctd_feedback;
    std::unique_ptr<uav_to_ros::ConversionServer<smarc_uavcan_messages_ThrusterFeedbackID, smarc_msgs::msg::ThrusterFeedback>> thruster1_feedback;
    std::unique_ptr<uav_to_ros::ConversionServer<smarc_uavcan_messages_ThrusterFeedbackID, smarc_msgs::msg::ThrusterFeedback>> thruster2_feedback;

};;


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

    imu_server = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_ahrs_Solution,sensor_msgs::msg::Imu>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::YOST_IMU_TOPIC);

    gps_fix = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_gnss_Fix,sensor_msgs::msg::NavSatFix>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::GPS_FIX_TOPIC);

    magnetic_field = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_ahrs_MagneticFieldStrength,sensor_msgs::msg::MagneticField>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::MAGNETIC_FIELD_TOPIC);

    sensor_pressure_bar30 = std::make_unique<uav_to_ros::ConversionServer<smarc_uavcan_messages_SensorPressureStamped,sensor_msgs::msg::FluidPressure>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::PRESS_DEPTH300_TOPIC,22);

    sensor_pressure_bar02 = std::make_unique<uav_to_ros::ConversionServer<smarc_uavcan_messages_SensorPressureStamped,sensor_msgs::msg::FluidPressure>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::PRESS_DEPTH20_TOPIC,23);

    motor_oil_pressure = std::make_unique<uav_to_ros::ConversionServer<smarc_uavcan_messages_SensorPressureStamped,sensor_msgs::msg::FluidPressure>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::MOTOR_OIL_PRESSURE_TOPIC,1);

    vbs_tank_pressure = std::make_unique<uav_to_ros::ConversionServer<smarc_uavcan_messages_SensorPressureStamped,sensor_msgs::msg::FluidPressure>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::VBS_TANK_PRESSURE_TOPIC,2);

    vbs_tank_temperature = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_device_Temperature,sensor_msgs::msg::Temperature>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::VBS_TANK_TEMPERATURE_TOPIC,2);

    motor_oil_temperature = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_device_Temperature,sensor_msgs::msg::Temperature>>(
    &canard_interface , shared_this,sam_msgs::msg::Topics::MOTOR_TEMP_TOPIC ,1);

    vbs_feedback = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_actuator_Status,sam_msgs::msg::PercentStamped>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::VBS_FB_TOPIC,13);
    
    lcg_feedback = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_actuator_Status,sam_msgs::msg::PercentStamped>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::LCG_FB_TOPIC,14);

    leak = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_actuator_Status,sam_msgs::msg::Leak>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::LEAK_TOPIC,200);

    // esc_status_server0 = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_esc_Status,uavcan_ros_msgs::msg::ESCStatus>>(
    // &canard_interface , shared_this, "esc_status0",0);

    // esc_status_server1 = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_esc_Status,uavcan_ros_msgs::msg::ESCStatus>>(
    // &canard_interface , shared_this, "esc_status1",1);

    circuit_status = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_power_CircuitStatus,sam_msgs::msg::CircuitStatusStamped>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::CIRCUIT_STATUS_TOPIC);

    battery_server2 = std::make_unique<uav_to_ros::ConversionServer<smarc_uavcan_messages_BatteryStateBasic,sensor_msgs::msg::BatteryState>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::BATTERY_STATUS_TOPIC);

    battery_server3 = std::make_unique<uav_to_ros::ConversionServer<uavcan_equipment_power_BatteryInfo,sensor_msgs::msg::BatteryState>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::BATTERY_STATUS_TOPIC);

    consumed_charge_array = std::make_unique<uav_to_ros::ConversionServer<smarc_uavcan_messages_ConsumedChargeArray,sam_msgs::msg::ConsumedChargeArray>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::CONSUMED_CHARED_ARRAY_TOPIC);

    ctd_feedback = std::make_unique<uav_to_ros::ConversionServer<smarc_uavcan_messages_CTDFeedback,smarc_msgs::msg::CTD>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::CTD_FB_TOPIC);

    thruster1_feedback = std::make_unique<uav_to_ros::ConversionServer<smarc_uavcan_messages_ThrusterFeedbackID,smarc_msgs::msg::ThrusterFeedback>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::THRUSTER1_FB_TOPIC,1);

    thruster2_feedback = std::make_unique<uav_to_ros::ConversionServer<smarc_uavcan_messages_ThrusterFeedbackID,smarc_msgs::msg::ThrusterFeedback>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::THRUSTER2_FB_TOPIC,2);

    panic_forwarding_server = std::make_unique<uav_to_ros::ConversionServer<uavcan_protocol_Panic,std_msgs::msg::String>>(
    &canard_interface , shared_this, sam_msgs::msg::Topics::PANIC_FB_TOPIC);




    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // Adjust the period as needed
        [this]() {
            canard_interface.process(1);
//             const uint64_t ts = micros64();
//             static uint64_t next_50hz_service_at = ts;
//             if (ts >= next_50hz_service_at) {
//                 next_50hz_service_at += 1000000ULL / 50U;
//                 this->send_NodeStatus();
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
    auto node = std::make_shared<UavcanToRosBridge>();
    node->start_canard_node();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}