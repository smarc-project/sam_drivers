#ifndef UAVCAN_ROS_BRIDGE_H
#define UAVCAN_ROS_BRIDGE_H

#include <canard_interface.h>
#include <dronecan_msgs.h>
#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>
#include <thread>
#include <condition_variable>
#ifndef CANARD_MTU_CAN_CLASSIC
#define CANARD_MTU_CAN_CLASSIC 8
#endif


namespace uav_to_ros {
rclcpp::Time inline convert_timestamp(const u_int64_t uav_time_usec)
{
    // rclcpp::Time ros_time;
    // ros_time.fromNSec(1000*uint64_t(uav_time.usec));
    // return ros_time;
    return rclcpp::Time(1000*static_cast<uint64_t>(uav_time_usec),RCL_ROS_TIME);
}
 
template <typename UAVMSG, typename ROSMSG>
bool convert(const UAVMSG&, std::shared_ptr<ROSMSG>)
{
    //ROS_WARN("Can't find conversion for uavcan type %s", uav_msg.getDataTypeFullName());
    static_assert(sizeof(UAVMSG) == -1 || sizeof(ROSMSG) == -1, "ERROR: You need to supply a convert specialization for the UAVCAN -> ROS msg types provided");
    return false;
}

template <typename UAVMSG ,typename ROSMSG>
bool convert(const UAVMSG& uav_msg, std::shared_ptr<ROSMSG> ros_msg, unsigned char uid)
{
    // uint8_t node_id = ;
    // if (uid == 255 || uid == node_id) {
        return convert(uav_msg, ros_msg);
    // }
    // return false;
}

template <typename UAVMSG, typename ROSMSG>//unsigned NodeMemoryPoolSize=16384
class ConversionServer {    
public:


    ConversionServer(CanardInterface* canard_interface, rclcpp::Node::SharedPtr ros_node, const std::string& ros_topic, unsigned char uid=255)
     : uav_node_(canard_interface), uid_(uid)
    {
        ros_pub_ = ros_node -> create_publisher<ROSMSG>(ros_topic, 10);
        
        //const int uav_sub_start_res = uav_sub.start(UavMsgCallbackBinder(this, &ConversionServer::conversion_callback));
    //     const int uav_sub_start_res = uav_sub.start(UavMsgExtendedBinder(this, &ConversionServer::conversion_callback));
    //     if (uav_sub_start_res < 0) {
    //         // ROS_ERROR("Failed to start the uav subscriber, error: %d", uav_sub_start_res);
            
            
    //     
    // }
    }

  
private:
    CanardInterface* uav_node_;
    void conversion_callback(const CanardRxTransfer& transfer, const UAVMSG& uav_msg);
    Canard::ObjCallback<ConversionServer,UAVMSG> sub_callback{this, &ConversionServer<UAVMSG,ROSMSG>::conversion_callback};
    Canard::Subscriber<UAVMSG> uav_sub_{sub_callback,0};

    rclcpp::Publisher<ROSMSG>::SharedPtr ros_pub_;
    unsigned char uid_;



};
template <typename UAVMSG, typename ROSMSG>
void ConversionServer<UAVMSG, ROSMSG>::conversion_callback(const CanardRxTransfer& transfer, const UAVMSG& uav_msg)
{
    auto ros_msg = std::make_shared<ROSMSG>();
    bool success = convert(uav_msg, ros_msg, uid_);
    if (success) {
        ros_pub_->publish(*ros_msg);
        
        /*
        else {
            ROS_WARN("There was an error trying to convert uavcan type %s", uav_msg.getDataTypeFullName());
        }
        */
        }
    }

};



namespace ros_to_uav {
    struct DefaultTag {};
    struct DVLTag {};
    struct LEDTag {};
    struct SSSTag {};
    // add more tags if the convertion uses the same variables as other convert fucntions

inline u_int64_t convert_timestamp(const rclcpp::Time& ros_time)
{
    u_int64_t uav_time_usec;
    uint64_t nsec = ros_time.nanoseconds();
    uav_time_usec = nsec / 1000;
    return uav_time_usec;
}

template <typename ROSMSG, typename UAVMSG>
bool convert(const std::shared_ptr<ROSMSG>, UAVMSG&)
{
    //ROS_WARN("Can't find conversion for uavcan type %s", uav_msg.getDataTypeFullName());
    static_assert(sizeof(UAVMSG) == -1 || sizeof(ROSMSG) == -1, "ERROR: You need to supply a convert specialization for the ROS -> UAVCAN msg types provided");
    return false;
}

template <typename ROSMSG, typename UAVMSG,typename TAG>
bool convert(const std::shared_ptr<ROSMSG> ros_msg, UAVMSG& uav_msg, unsigned char, TAG )
{
    return convert(ros_msg, uav_msg);
}


template <typename ROSREQ, typename UAVREQ>
bool convert_request(const std::shared_ptr<ROSREQ>, UAVREQ&)
{
    static_assert(sizeof(UAVREQ) == -1 || sizeof(ROSREQ) == -1, "ERROR: You need to supply a convert specialization for the ROS -> UAVCAN service request");
    return false;
}

template <typename UAVRES, typename ROSRES>
bool convert_response(const UAVRES&, std::shared_ptr<ROSRES>)
{
    static_assert(sizeof(UAVRES) == -1 || sizeof(ROSRES) == -1, "ERROR: You need to supply a convert specialization for the UAVCAN -> ROS service response");
    return false;
}

template <typename UAVMSG, typename ROSMSG, typename TAG = DefaultTag>
class ConversionServer {
public:
    ConversionServer(CanardInterface* canard_interface, rclcpp::Node::SharedPtr ros_node, const std::string& ros_topic, unsigned char uid=0, TAG tag = DefaultTag{}) :
     uav_node_(canard_interface), uid_(uid), running_(true), transfer_id_(0), tag_(tag)
    {
        
        // const int uav_pub_init_res = uav_pub.init();
        // if (uav_pub_init_res < 0) {
        //     // ROS_ERROR("Failed to start the uav publisher, error: %d, type: %s", uav_pub_init_res, UAVMSG::getDataTypeFullName());
        //     RCLCPP_ERROR(ros_node->get_logger(), "Failed to start the uav publisher, error: %d, type: %s", uav_pub_init_res, UAVMSG::getDataTypeFullName());
        // }

        ros_sub_ = ros_node->create_subscription<ROSMSG>(ros_topic, 10, 
            std::bind(&ConversionServer::conversion_callback, this, std::placeholders::_1));

        std::string service_name = ros_sub_->get_topic_name();
        std::replace(service_name.begin(), service_name.end(), '/', '_');
        service_name = std::string("start_stop") + service_name;
        ros_service_ = ros_node->create_service<std_srvs::srv::SetBool>(service_name, 
            std::bind(&ConversionServer::start_stop, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(ros_node->get_logger(), "Announcing service: %s", service_name.c_str());
    }

    bool start_stop(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res)
    {
        running_ = req->data;
        res->success = true;
        if (!running_) {
            res->message = std::string("Stopping subscriber ") + ros_sub_-> get_topic_name();
        }
        else {
            res->message = std::string("Starting subscriber ") + ros_sub_-> get_topic_name();
        }
        return true;
    }

    void conversion_callback(const std::shared_ptr<ROSMSG> ros_msg)
    {
        if (!running_) {
            return;
        }
        UAVMSG uav_msg;
        bool success = convert(ros_msg, uav_msg, uid_, tag_);
        if (success) {
            canard_publisher.broadcast(uav_msg);
        }


    }

private:
    CanardInterface* uav_node_;
    rclcpp::Subscription<ROSMSG>::SharedPtr ros_sub_;
    Canard::Publisher<UAVMSG> canard_publisher{*uav_node_};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ros_service_;
    // Canard::Publisher<UAVMSG> uav_pub_(uav_node_);
    unsigned char uid_;
    bool running_;
    uint8_t transfer_id_;
    TAG tag_;
};

template <typename UAVSRV_REQUEST, typename UAVSRV_RESPONSE, typename ROSSRV>
class ServiceConversionServer {
public:
    ServiceConversionServer(
        CanardInterface* canard_interface,
        rclcpp::Node::SharedPtr ros_node,
        const std::string& ros_service_name)
        : uav_node_(canard_interface), ros_node_(ros_node) {
        ros_service_ = ros_node_->create_service<ROSSRV>(
            ros_service_name,
            std::bind(
                &ServiceConversionServer::service_callback,
                this,
                std::placeholders::_1,  
                std::placeholders::_2,  
                std::placeholders::_3   
            )
        );
    }

void service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<typename ROSSRV::Request> ros_req,
    std::shared_ptr<typename ROSSRV::Response> ros_res)
{

    UAVSRV_REQUEST uav_req;
    bool success = convert_request(ros_req, uav_req);
    if (!success) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to convert ROS2 request to UAVCAN request.");
        ros_service_->send_response(*request_header, *ros_res);
        return;
    }

    // Prepare a flag to indicate when the response is received
    bool response_received = false;

    auto response_callback = [&, ros_res](const CanardRxTransfer& transfer, const UAVSRV_RESPONSE& uav_res) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received UAVCAN response");

        if (!convert_response(uav_res, ros_res)) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to convert UAVCAN response to ROS2 response.");
        }
        

        response_received = true; 
    };

    callback_ = std::unique_ptr<Canard::LambdaCallback<UAVSRV_RESPONSE>>(
        Canard::allocate_lambda_callback<UAVSRV_RESPONSE>(response_callback)
    );

    uav_client_ = std::make_unique<Canard::Client<UAVSRV_RESPONSE>>(*uav_node_, *callback_);

    bool request_success = uav_client_->request(ros_req->node_id, uav_req);
    if (!request_success) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to perform service call.");
        ros_service_->send_response(*request_header, *ros_res);
        return;
    }

    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5); 
    while(!response_received) {
        uav_node_->process(1); 

        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Timeout waiting for UAVCAN response.");
            break;
        }
    }


    ros_service_->send_response(*request_header, *ros_res);
}

private:
    CanardInterface* uav_node_;
    std::unique_ptr<Canard::Client<UAVSRV_RESPONSE>> uav_client_;  
    std::unique_ptr<Canard::LambdaCallback<UAVSRV_RESPONSE>> callback_;  
    typename rclcpp::Service<ROSSRV>::SharedPtr ros_service_;
    rclcpp::Node::SharedPtr ros_node_;

    std::shared_ptr<rmw_request_id_t> request_header_;
    std::shared_ptr<typename ROSSRV::Response> ros_res_;
};

};
#endif // UAVCAN_ROS_BRIDGE

