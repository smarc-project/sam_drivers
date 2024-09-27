#ifndef CANARD_INTERFACE_H
#define CANARD_INTERFACE_H

#include <canard.h>
#include <time_utils.h>
#include <cstdio>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.GetNodeInfo.h> 
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <canard/handler_list.h>
#include <canard/transfer_object.h>
#include <socketcan.h>

class CanardInterface : public Canard::Interface {
    friend class RosToUavcanBridge;
    friend class ServiceConversionBridge;
    friend class UavcanToRosBridge;
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


    CanardCANFrame* peekTxQueue() const {
    return canardPeekTxQueue(&canard);
    }
private:
    CanardInstance canard;
    SocketCANInstance socketcan;
    CanardTxTransfer tx_transfer;
    uint8_t memory_pool[2048];
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub;
    uint64_t next_1hz_service_at;
};

#endif // CANARD_INTERFACE_H