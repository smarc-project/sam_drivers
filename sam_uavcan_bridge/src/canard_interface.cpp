#include <canard_interface.h>
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
