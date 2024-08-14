#ifndef UAVCAN_ROS_BRIDGE_H
#define UAVCAN_ROS_BRIDGE_H

#include <dronecan_msgs.h>
#include <canard.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.GetNodeInfo.h> 
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <canard/handler_list.h>
#include <canard/transfer_object.h>
#include <socketcan.h>
#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/set_bool.hpp"

#ifndef CANARD_MTU_CAN_CLASSIC
#define CANARD_MTU_CAN_CLASSIC 8
#endif
class CanardInterface : public Canard::Interface {
    friend class RosToUavcanBridge;
    CanardInterface(uint8_t iface_index): Interface(iface_index), node_status_pub(*this) {};
public:


    void init(const char* interface_name, uint8_t node_id);
    void process(uint32_t timeout_ms);

    bool broadcast(const Canard::Transfer& transfer) override;
    bool request(uint8_t destination_node_id, const Canard::Transfer& transfer) override;
    bool respond(uint8_t destination_node_id, const Canard::Transfer& transfer) override;
    uint8_t get_node_id() const override { return canard.node_id; }
    void set_node_id(uint8_t node_id) {
        canardSetLocalNodeID(&canard, node_id);
    }
    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);



private:
    CanardInstance canard;
    SocketCANInstance socketcan;
    CanardTxTransfer tx_transfer;
    uint8_t memory_pool[2048];
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub;
    uint64_t next_1hz_service_at;
};

// namespace uav_to_ros {
// rclcpp::Time convert_timestamp(const u_int64_t uav_time_usec)
// {
//     // rclcpp::Time ros_time;
//     // ros_time.fromNSec(1000*uint64_t(uav_time.usec));
//     // return ros_time;
//     return rclcpp::Time(1000*static_cast<uint64_t>(uav_time_usec),RCL_ROS_TIME);
// }
 
// template <typename UAVMSG, typename ROSMSG>
// bool convert(const UAVMSG&, ROSMSG&)
// {
//     //ROS_WARN("Can't find conversion for uavcan type %s", uav_msg.getDataTypeFullName());
//     static_assert(sizeof(UAVMSG) == -1 || sizeof(ROSMSG) == -1, "ERROR: You need to supply a convert specialization for the UAVCAN -> ROS msg types provided");
//     return false;
// }

// template <typename ROSMSG>
// bool convert(const CanardRxTransfer& transfer, ROSMSG& ros_msg, unsigned char uid)
// {
//     uint8_t node_id = transfer.source_node_id;
//     if (uid == 255 || uid == node_id) {
//         return convert(transfer, ros_msg);
//     }
//     return false;
// }

// template <typename UAVMSG, typename ROSMSG>//unsigned NodeMemoryPoolSize=16384
// class ConversionServer {
// public:
//     // typedef uavcan::ReceivedDataStructure<UAVMSG> ReceivedDataStructure;
//     // using ReceivedDataStructure = UAVMSG;
//     // typedef uavcan::Node<NodeMemoryPoolSize> UavNode;
//     using UavNode = CanardInstance*; //uavcan::Node<NodeMemoryPoolSize>;
//     // typedef uavcan::MethodBinder<ConversionServer*, void (ConversionServer::*)(const UAVMSG&) const> UavMsgCallbackBinder;
//     // using UavMsgCallbackBinder = uavcan::MethodBinder<ConversionServer*, void (ConversionServer::*)(const UAVMSG&) const>;
//     // typedef uavcan::MethodBinder<ConversionServer*, void (ConversionServer::*)(const ReceivedDataStructure&) const> UavMsgExtendedBinder;
//     // using UavMsgExtendedBinder = uavcan::MethodBinder<ConversionServer*, void (ConversionServer::*)(const ReceivedDataStructure&) const>;
//     //uavcan::Subscriber<UAVMSG, UavMsgCallbackBinder> uav_sub;
//     // uavcan::Subscriber<UAVMSG, UavMsgExtendedBinder> uav_sub;
//     // Canard::Subscriber<UAVMSG> uav_sub;
//     // // ros::Publisher ros_pub;
//     // rclcpp::Publisher<ROSMSG>::SharedPtr ros_pub;
//     // CanardInstances& uav_node;
//     // unsigned char uid;

//     ConversionServer(UavNode uav_node, rclcpp::Node::SharedPtr ros_node, const std::string& ros_topic, unsigned char uid=255)
//      : uav_node_(uav_node), uid_(uid)
//     {
//         ros_pub_ = ros_node -> create_publisher<ROSMSG>(ros_topic, 10);

//         //const int uav_sub_start_res = uav_sub.start(UavMsgCallbackBinder(this, &ConversionServer::conversion_callback));
//         const int uav_sub_start_res = uav_sub.start(UavMsgExtendedBinder(this, &ConversionServer::conversion_callback));
//         if (uav_sub_start_res < 0) {
//             // ROS_ERROR("Failed to start the uav subscriber, error: %d", uav_sub_start_res);
            
            
//         }
//     }

//     void conversion_callback(const uav_msg) const
//     {
//         ROSMSG ros_msg;
//         bool success = convert(uav_msg, ros_msg, uid_);
//         if (success) {
//             ros_pub->publish(ros_msg);
//         }
//         /*
//         else {
//             ROS_WARN("There was an error trying to convert uavcan type %s", uav_msg.getDataTypeFullName());
//         }
//         */
//     }
// private:
//     UavNode uav_node_;
//     Canard::Subscriber<UAVMSG> uav_sub_;
//     typename rclcpp::Publisher<ROSMSG>::SharedPtr ros_pub_;
//     unsigned char uid_;


// };

// }

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
bool convert_request(const ROSREQ&, UAVREQ&)
{
    static_assert(sizeof(UAVREQ) == -1 || sizeof(ROSREQ) == -1, "ERROR: You need to supply a convert specialization for the ROS -> UAVCAN service request");
    return false;
}

template <typename UAVRES, typename ROSRES>
bool convert_response(const UAVRES&, ROSRES&)
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
    typename rclcpp::Subscription<ROSMSG>::SharedPtr ros_sub_;
    Canard::Publisher<UAVMSG> canard_publisher{*uav_node_};
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ros_service_;
    // Canard::Publisher<UAVMSG> uav_pub_(uav_node_);
    unsigned char uid_;
    bool running_;
    uint8_t transfer_id_;
    TAG tag_;
};

// template <typename UAVSRV, typename ROSSRV, unsigned NodeMemoryPoolSize=16384>
// class ServiceConversionServer {
// public:
//     // typedef uavcan::Node<NodeMemoryPoolSize> UavNode;
//     using UavNode = CanardInstance*; //uavcan::Node<NodeMemoryPoolSize>;

//     ServiceConversionServer(UavNode uav_node, rclcpp::Node::SharedPtr ros_node, const std::string& ros_service_name)
//      : uav_node_(uav_node), transfer_id_(0) //uav_client_(uav_node) //, node_id(node_id)
//     {
// 		// const int uav_client_init_res = uav_client.init();
// 		// if (uav_client_init_res < 0) {
//         //     // ROS_ERROR("Failed to start the uav publisher, error: %d, type: %s", uav_client_init_res, UAVSRV::getDataTypeFullName());
//         //      RCLCPP_ERROR(ros_node->get_logger(), "Failed to start the uav publisher, error: %d, type: %s", uav_client_init_res, UAVSRV::getDataTypeFullName());
        
// 		// }
// 		// uav_client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(200));
// 		// uav_client.setPriority(uavcan::TransferPriority::OneHigherThanLowest);

// 		ros_service_ = ros_node-> create_service<ROSSRV>(ros_service_name, std::bind(&ServiceConversionServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));
//     }

//     bool service_callback(const std::shared_ptr<typename ROSSRV::Request> ros_req, typename std::shared_ptr<typename ROSSRV::Respons> ros_res)
//     {
//         typename UAVSRV::Request uav_req;
//         bool success = convert_request(*ros_req, uav_req);
//         if (!success) {
//             // ROS_WARN("There was an error trying to convert uavcan service %s", UAVSRV::getDataTypeFullName());
//             RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "There was an error trying to convert uavcan service %s", UAVSRV::getDataTypeFullName());
//             return false;
//         }

// 		// uav_client.setCallback([&](const uavcan::ServiceCallResult<UAVSRV>& uav_res) {
//         //     if (uav_res.isSuccessful()) {
//         //         success = convert_response(static_cast<const typename UAVSRV::Response&>(uav_res.getResponse()), *ros_res);
//         //     }
//         //     else {
//         //         // ROS_WARN("There was an error call the uavcan service %s on node id %d", UAVSRV::getDataTypeFullName(), static_cast<int>(uav_res.getCallID().server_node_id.get()));
//         //         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "There was an error call the uavcan service %s on node id %d", UAVSRV::getDataTypeFullName(), static_cast<int>(uav_res.getCallID().server_node_id.get()));
//         //         success = false;
// 		// 	}
//         // });
//         uint8_t buffer[CANARD_MTU_CAN_CLASSIC];
//         size_t total_size = serialize_uavcan_request(uav_req, buffer);
//         int32_t result = canardRequest(uav_node_,
//                                       UAVSRV::Request::FIXED_PORT_ID,  // transfer_kind (adjust as needed)
//                                       UAVSRV::Request::DATA_TYPE_SIGNATURE,  // data_type_signature (adjust as needed)
//                                       &transfer_id_,
//                                       CANARD_TRANSFER_PRIORITY_MEDIUM,
//                                       buffer,
//                                       total_size);
//         // const int uav_call_res = uav_client.call(ros_req.node_id, uav_req);
// 		if (result < 0) {
//             // ROS_WARN("Unable to perform service call: %d", uav_call_res);
//             RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unable to perform service call: %d", result);
//             return false;
// 		}

//         // uav_node.setModeOperational();
//         // while (uav_client.hasPendingCalls()) {
//         //     const int res = uav_node.spin(uavcan::MonotonicDuration::fromMSec(10));
//         //     if (res < 0) {
//         //         // ROS_WARN("Transient failure: %d", res);
//         //         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Transient failure: %d", res);
//         //         success = false;
//         //         break;
//         //     }
//         // }

//         return success;
//     }

    
// private:
// UavNode uav_node_;
// rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ros_service_;
// unsigned char uid_;
// bool running_;
// uint8_t transfer_id_;

// size_t serialize_uavcan_request(const typename UAVSRV::Request& req, uint8_t* buffer)
//     {
//         // Implement serialization logic here
//         // This will depend on your specific UAVSRV::Request type
//         return 0;  // Placeholder return
//     }

// };
}
#endif // UAVCAN_ROS_BRIDGE

