// #include "RadioLib.h"
#include "EspHal.h"
#include "LoraMesher.h"
#include "esp_log.h"
#include "conf.h"
#include "messages.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "freertos/queue.h"
#include "lora_tasks.h"
#include "tinysync.h"
#include "esp_random.h"
#include "dw3000_port.h"

static const char *TAG = "LORA_TASKS";
EspHal* hal = nullptr;
LoraMesher& radio = LoraMesher::getInstance();
// Task handle for receiving LoRa messages
TaskHandle_t loraReceiveTaskHandle = NULL;

// message queues
static QueueHandle_t Ping_Q;
static QueueHandle_t requwb_Q;
static QueueHandle_t receive_Q;
static QueueHandle_t emit_lora_Q;
static QueueHandle_t ranging_request_Q;

/**
 * @brief Encode a message into a BaseMessage
 * 
 * @param buffer Buffer to hold the encoded message
 * @param bufferSize Size of the buffer
 * @param msgType Type of the message
 * @param msg Pointer to the message to encode
 * @param msgSize Size of the message (optional, default is 0)
 * @return int Size of the encoded message, or -1 on error
 */
int encodePBMessage(uint8_t *buffer, size_t bufferSize, MessageType msgType, const void *msg, int msgSize=0) {
    // Create an output stream that will hold the encoded BaseMessage.
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, bufferSize);
    bool status = false;

    // Initialize the base message to default values.
    BaseMessage base = BaseMessage_init_default;
    base.type = msgType;
    
    // Prepare a temporary buffer for the payload.
    uint8_t payloadBuffer[128];
    pb_ostream_t payloadStream = pb_ostream_from_buffer(payloadBuffer, sizeof(payloadBuffer));

    // Encode the inner message into the temporary buffer based on the type.
    switch (msgType) {
        case MessageType::MessageType_PING:
            status = pb_encode(&payloadStream, Ping_fields, (const Ping*)msg);
            break;
        case MessageType::MessageType_TWRTS:
            status = pb_encode(&payloadStream, Twrts_fields, (const Twrts*)msg);
            break;
        case MessageType::MessageType_REQUWB:
            status = pb_encode(&payloadStream, Requwb_fields, (const Requwb*)msg);
            break;
        case MessageType::MessageType_OTHER:
            memcpy(payloadBuffer, msg, msgSize);
            break;
        default:
            ESP_LOGE(TAG, "Unknown message type");
            return -1;
    }

    if (!status) {
        ESP_LOGE(TAG, "Encoding payload failed: %s", PB_GET_ERROR(&payloadStream));
        return -1;
    }
    
    // Check that the payload fits in the BaseMessage payload buffer.
    if (payloadStream.bytes_written > sizeof(base.payload.bytes)) {
        ESP_LOGE(TAG, "Encoded payload is too large: %d bytes", (int)payloadStream.bytes_written);
        return -1;
    }
    
    // Copy the encoded payload into the base message.
    base.payload.size = payloadStream.bytes_written;
    memcpy(base.payload.bytes, payloadBuffer, payloadStream.bytes_written);

    // Now encode the base message (which includes the payload as a bytes field).
    status = pb_encode(&stream, BaseMessage_fields, &base);
    if (!status) {
        ESP_LOGE(TAG, "Encoding BaseMessage failed: %s", PB_GET_ERROR(&stream));
        return -1;
    }
    
    return stream.bytes_written;
}


/**
 * @brief Task to process received packets from LoRaMesher
 * 
 * @param pvParameters Task parameters (unused)
 */
void loraReceiveTask(void* pvParameters) {
    // init the reception queue
    
    // decoding buffer
    uint8_t buffer[128];
    for (;;) {
        // Wait for notification
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);

        // Process all received packets
        while (radio.getReceivedQueueSize() > 0) {
            
            // Retrieve the next packet
            AppPacket<uint8_t>* packet = radio.getNextAppPacket<uint8_t>();
            // Decode the BaseMessage
            pb_istream_t stream = pb_istream_from_buffer(packet->payload, packet->payloadSize);
            BaseMessage base = BaseMessage_init_default;
            if (!pb_decode(&stream, BaseMessage_fields, &base)) {
                ESP_LOGE(TAG, "Failed to decode BaseMessage: %s", PB_GET_ERROR(&stream));
                radio.deletePacket(packet);
                continue;
            }
            // send to appropriate handler
            pb_istream_t payloadStream = pb_istream_from_buffer(base.payload.bytes, base.payload.size);
            switch (base.type) {
                case MessageType_PING: {
                    Ping ping = Ping_init_default;
                    if (!pb_decode(&payloadStream, Ping_fields, &ping)) {
                        ESP_LOGE(TAG, "Failed to decode Ping: %s", PB_GET_ERROR(&payloadStream));
                        radio.deletePacket(packet);
                        continue;
                    }
                    // send to queue
                    xQueueSend(Ping_Q, &ping, 0);
                    break;
                }
                case MessageType_TWRTS: {
                    Twrts twrts = Twrts_init_default;
                    if (!pb_decode(&payloadStream, Twrts_fields, &twrts)) {
                        ESP_LOGE(TAG, "Failed to decode Twrts: %s", PB_GET_ERROR(&payloadStream));
                        radio.deletePacket(packet);
                        continue;
                    }
                    // send to queue
                    twrts_decode_push(twrts);
                    break;
                }
                case MessageType_REQUWB: {
                    Requwb requwb = Requwb_init_zero;
                    if (!pb_decode(&payloadStream, Requwb_fields, &requwb)) {
                        ESP_LOGE(TAG, "Failed to decode Requwb: %s", PB_GET_ERROR(&payloadStream));
                        radio.deletePacket(packet);
                        continue;
                    }
                    // ESP_LOGI(TAG, "Requwb received with AN: %d", requwb.AN);
                    // send to queue
                    xQueueSend(requwb_Q, &requwb, 0);
                    break;
                }
                default:
                    ESP_LOGE(TAG, "Unknown message type");
            }
            // Print the received packet's payload
            // printDataPacket(packet);

            // Release packet memory
            radio.deletePacket(packet);
        }
    }
}

void log_info() {
    // Create a copy of the routing table
    LM_LinkedList<RouteNode>* routingTableList = radio.routingTableListCopy();

    ESP_LOGI(TAG, "local addr: %X", radio.getLocalAddress());
    // Print general information
    ESP_LOGI(TAG, "==================================");
    // Format and log the routing table
    ESP_LOGI(TAG, "Routing Table:");
    routingTableList->setInUse();
    for (int i = 0; i < radio.routingTableSize(); i++) {
        RouteNode* rNode = (*routingTableList)[i];
        NetworkNode node = rNode->networkNode;
        ESP_LOGI(TAG, "|%X(%d)->%X", node.address, (int)node.metric, rNode->via);
    }
    ESP_LOGI(TAG, "==================================");

    // Release and clean up routing table list
    routingTableList->releaseInUse();
    delete routingTableList;
}

void log_task(void *pvParameters) {
    for(;;){
        log_info();
        vTaskDelay(100000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void ping_send_task(void *pvParameters) {
    // Create a ping packet
    int counter= 1;
    Ping ping = Ping_init_default;
    uint8_t text_buffer[128];
    for(;;){
        counter = counter % 256;
        ping.counter = counter;
        // Encode the ping message
        int msgSize = encodePBMessage(text_buffer, sizeof(text_buffer), MessageType_PING, &ping);
        if (msgSize < 0) {
            ESP_LOGE(TAG, "Failed to encode ping message %d", msgSize);
            continue;
        }
        // Send the ping packet to the broadcast address
        radio.createPacketAndSend(BROADCAST_ADDR, text_buffer, msgSize);
        ESP_LOGI(TAG, "Ping sent %d", counter);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}

void ping_recv_task(void *pvParameters) {
    Ping ping;
    for(;;){
        if (xQueueReceive(Ping_Q, &ping, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Ping received with counter: %" PRIu32, ping.counter);
        }
    }
    vTaskDelete(NULL);
}

void requwb_recv_task(void *pvParameters) {
    Requwb requwb;
    for(;;){
        if (xQueueReceive(requwb_Q, &requwb, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Requwb received with AN: %" PRIu32, requwb.AN);
        }
    }
    vTaskDelete(NULL);
}

bool is_requestQEmpty(){
    int n = uxQueueMessagesWaiting(ranging_request_Q);
    return (n == 0);
}

void waitLoraCalm(){
    while((radio.getReceivedQueueSize() != 0) && radio.getSendQueueSize()!=0){
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


void requwb_coordinator(void *pvParameters) {
    Requwb requwb;
    bool wait_for_response = false;
    const int interval = 8*10000/2; // in ms ? REQUESTED_RANGING_AMOUNT
    int req_amount = REQUESTED_RANGING_AMOUNT;
    int response_delay = 1000; // in ms
    for(;;){
        if (xQueueReceive(requwb_Q, &requwb, interval) == pdTRUE) {
            // we received a request
            // check that we are free
            if(!is_requestQEmpty()){
                continue;
            }
            waitLoraCalm();
            req_amount = requwb.amount;
            response_delay = requwb.delay;
            // accept the request
            ESP_LOGI(TAG, "Requwb received with AN: %" PRIu32, requwb.AN);
            if(!requwb.is_response){

                requwb.AN = radio.getLocalAddress();
                requwb.delay = 1000;
                requwb.SN = esp_random() % 256;
                requwb.is_response = true;

                // Encode the requwb message
                static uint8_t text_buffer[64];
                int msgSize = encodePBMessage(text_buffer, sizeof(text_buffer), MessageType_REQUWB, &requwb);
                if (msgSize < 0) {
                    ESP_LOGE(TAG, "Failed to encode requwb message %d", msgSize);
                    continue;
                }
                ESP_LOGI(TAG, "Requwb response with SN: %" PRIu32, requwb.SN);
                // Send the requwb packet to the broadcast address
                radio.createPacketAndSend(BROADCAST_ADDR, text_buffer, msgSize);
                setFakeEvent(false);
                vTaskDelay(pdMS_TO_TICKS(response_delay));
                xQueueSend(ranging_request_Q, &req_amount, 0); 
            } else{
                setFakeEvent(false);
                vTaskDelay(pdMS_TO_TICKS(response_delay-300));
                xQueueSend(ranging_request_Q, &req_amount, 0); 
                }
        } else {
            // we should initiate a request
            if(!is_requestQEmpty()){
                continue;
            }
            waitLoraCalm();
            ESP_LOGI(TAG, "---------------------- Requwb timeout");
            requwb.AN = radio.getLocalAddress();
            requwb.delay = 1000;
            requwb.SN = esp_random() % 256;
            requwb.is_response = false;
            requwb.amount = req_amount;
            // Encode the requwb message
            static uint8_t text_buffer[64];
            int msgSize = encodePBMessage(text_buffer, sizeof(text_buffer), MessageType_REQUWB, &requwb);
            if (msgSize < 0) {
                ESP_LOGE(TAG, "Failed to encode requwb message %d", msgSize);
                continue;
            }
            // Send the requwb packet to the broadcast address
            radio.createPacketAndSend(BROADCAST_ADDR, text_buffer, msgSize);
        }
    }
    vTaskDelete(NULL);
}

void twrts_decode_push(Twrts twrts) {
    time_stamps_t time_stamps;
    // ESP_LOGI(TAG, "Twrts received with SN: %" PRIu32, twrts.SN);
    time_stamps.poll_tx.DW3000_time = twrts.uwb_poll_tx;
    time_stamps.poll_tx.MCU_time = twrts.mcu_poll_tx;
    time_stamps.resp_rx.DW3000_time = twrts.uwb_resp_rx;
    time_stamps.resp_rx.MCU_time = twrts.mcu_resp_rx;
    time_stamps.resp_tx.DW3000_time = twrts.uwb_resp_tx;
    time_stamps.resp_tx.MCU_time = twrts.mcu_resp_tx;
    time_stamps.poll_rx.DW3000_time = twrts.uwb_poll_rx;
    time_stamps.poll_rx.MCU_time = twrts.mcu_poll_rx;
    time_stamps.final_tx.DW3000_time = twrts.uwb_final_tx;
    time_stamps.final_tx.MCU_time = twrts.mcu_final_tx;
    time_stamps.final_rx.DW3000_time = twrts.uwb_final_rx;
    time_stamps.final_rx.MCU_time = twrts.mcu_final_rx;
    time_stamps.SN = twrts.SN;
    time_stamps.other_addr = twrts.AN;
    time_stamps.distance = twrts.distance;
    time_stamps.is_initiator = twrts.is_initiator;
    time_stamps.is_SS = twrts.is_SS;
    xQueueSend(receive_Q, &time_stamps, 0);
}

void send_twrts(time_stamps_t time_stamps) {
    Twrts twrts = Twrts_init_default;
    twrts.uwb_poll_tx = time_stamps.poll_tx.DW3000_time;
    twrts.mcu_poll_tx = time_stamps.poll_tx.MCU_time;
    twrts.uwb_resp_rx = time_stamps.resp_rx.DW3000_time;
    twrts.mcu_resp_rx = time_stamps.resp_rx.MCU_time;
    twrts.uwb_resp_tx = time_stamps.resp_tx.DW3000_time;
    twrts.mcu_resp_tx = time_stamps.resp_tx.MCU_time;
    twrts.uwb_poll_rx = time_stamps.poll_rx.DW3000_time;
    twrts.mcu_poll_rx = time_stamps.poll_rx.MCU_time;
    twrts.uwb_final_tx = time_stamps.final_tx.DW3000_time;
    twrts.mcu_final_tx = time_stamps.final_tx.MCU_time;
    twrts.uwb_final_rx = time_stamps.final_rx.DW3000_time;
    twrts.mcu_final_rx = time_stamps.final_rx.MCU_time;
    twrts.SN = time_stamps.SN;
    twrts.AN = time_stamps.other_addr;
    twrts.distance = time_stamps.distance;
    twrts.is_initiator = time_stamps.is_initiator;
    twrts.is_SS = time_stamps.is_SS;
    // Encode the twrts message
    static uint8_t text_buffer[128];
    int msgSize = encodePBMessage(text_buffer, sizeof(text_buffer), MessageType_TWRTS, &twrts);
    if (msgSize < 0) {
        ESP_LOGE(TAG, "Failed to encode twrts message %d", msgSize);
        return;
    }
    // Send the twrts packet to the broadcast address
    radio.createPacketAndSend(BROADCAST_ADDR, text_buffer, msgSize);
    // radio.sendReliable(BROADCAST_ADDR, text_buffer, msgSize);
}

void twrts_send_task(void *pvParameters) {
    time_stamps_t time_stamps;
    int sent_counter = 0;
    for(;;){
        if(xQueuePeek(emit_lora_Q, &time_stamps, portMAX_DELAY) == pdTRUE) {
            // get the number of msgs in the request queue
            int n = uxQueueMessagesWaiting(ranging_request_Q);
            if(n > 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
            } else if(xQueueReceive(emit_lora_Q, &time_stamps, 0)){
                // get the number of msgs in the request queue
                sent_counter++;
                int n = uxQueueMessagesWaiting(ranging_request_Q);
                // ESP_LOGI(TAG, "n requests in the queue : %d", n);
                // ESP_LOGI(TAG, "Twrts sent SN: %" PRIu8, time_stamps.SN);
                // ESP_LOGI(TAG, "Twrts sent counter: %d", sent_counter);
                send_twrts(time_stamps);
            }
        }
    }
    vTaskDelete(NULL);
}



void lora_setup(QueueHandle_t Q_receive_lora, QueueHandle_t Q_emit_lora, QueueHandle_t Q_ranging_request)
{

    Ping_Q = xQueueCreate(3, sizeof(Ping));
    requwb_Q = xQueueCreate(3, sizeof(Requwb));
    receive_Q = Q_receive_lora;
    emit_lora_Q = Q_emit_lora;
    ranging_request_Q = Q_ranging_request;
    
    // Initialize the custom HAL
    hal = new EspHal(HSPI_SCLK_GPIO, HSPI_MISO_GPIO, HSPI_MOSI_GPIO, HSPI_HOST);//local_spi_handle
    // SX1276 radio = new Module(hal, LORA_CS_GPIO, LORA_IRQ_GPIO, LORA_RST_GPIO, LORA_GP1_GPIO);
    ESP_LOGI(TAG, "Initializing HAL");
    // Configure LoraMesher with custom HAL
    LoraMesher::LoraMesherConfig config;
    config.hal = hal;  // Pass the custom HAL
    config.loraCs = LORA_CS_GPIO;
    config.loraIrq = LORA_IRQ_GPIO;
    config.loraRst = LORA_RST_GPIO;
    config.loraIo1 = LORA_GP1_GPIO;
    config.freq = 915.0; // Frequency in MHz for NA LoRa band
    config.module = LoraMesher::LoraModules::SX1276_MOD;
    ESP_LOGI(TAG, "beginnign radio:");
    // Initialize LoraMesher
    radio.begin(config);
    ESP_LOGI(TAG, "Radio Initialized Successfully!");
    // Start LoraMesher
    radio.start();
    ESP_LOGI(TAG, "LoRa Initialized Successfully!");
    // Create LoRa receive task
    BaseType_t res = xTaskCreate( loraReceiveTask, "LoRa Receive Task", 4096, NULL, 10, &loraReceiveTaskHandle);

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LoRa Receive Task: %d", res);
    } else {
        // Link the task to LoRaMesher for notifications
        radio.setReceiveAppDataTaskHandle(loraReceiveTaskHandle);
        // xTaskCreate(ping_send_task, "ping tx Task", 4096, NULL, 1, NULL);
        // xTaskCreate(log_task, "log Task", 4096, NULL, 1, NULL);
        // xTaskCreate(ping_recv_task, "ping rx Task", 4096, NULL, 1, NULL);
        // xTaskCreate(requwb_recv_task, "requwb rx Task", 2048, NULL, 1, NULL);
        // xTaskCreate(twrts_recv_task, "twrts rx Task", 2048, NULL, 5, NULL);
        xTaskCreate(requwb_coordinator, "requwb coordinator Task", 4096, NULL, 2, NULL);
        xTaskCreate(twrts_send_task, "twrts tx Task", 4096, NULL, 6, NULL);
        ESP_LOGI(TAG, "LoRa Receive Tasks Created Successfully!");
    }
}
