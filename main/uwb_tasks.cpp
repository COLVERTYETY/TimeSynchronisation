#include "uwb_tasks.h"
#include "driver/spi_master.h"

#include "dw3000.h"
#include "dw3000_shared_functions.h"
#include "esp_log.h"
#include "conf.h"
#include "esp_timer.h"
#include "messages.pb.h"
#include "pb_encode.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "tinysync.h"
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"

// #include <soc/rtc_wdt.h>

#include "dw3000_mac_802_15_4.h"

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_TX_TO_RESP_RX_DLY_UUS 600 // Delay from end of transmission to activation of reception //! arbitrary value
#define POLL_RX_TO_RESP_TX_DLY_UUS 600 // Delay from end of reception to start of transmission      //! arbitrary value
#define LONG_RX_TIMEOUT_UUS  100000 //  Receive response timeout, expressed in UWB microseconds.
#define SHORT_RX_TIMEOUT_UUS 100000 //  Receive response timeout, expressed in UWB microseconds.


#define FC_IDX 0 // index of the frame control in the message
#define FC2_IDX 1 // index of the frame control in the message
#define ALL_MSG_SN_IDX 2 // index of the sequence number in the message
#define PAN_ID_HI_IDX 3 // HI index of the PAN ID in the message
#define PAN_ID_LO_IDX 4 // LO index of the PAN ID in the message
#define DEST_ID_HI_IDX 5 // HI index of the init ID in the message
#define DEST_ID_LO_IDX 6 // LO index of the init ID in the message
#define SOURCE_ID_HI_IDX 7 // HI index of the responder ID in the message
#define SOURCE_ID_LO_IDX 8 // LO index of the responder ID in the message
#define FUNCTION_CODE_IDX 9 // index of the function code in the message
#define ACTIVITY_CODE_IDX 10 // index of the activity code in the message

#define SS_RESP_MSG_POLL_RX_TS_IDX  10  // time stamp index
#define SS_RESP_MSG_RESP_TX_TS_IDX  14  // time stamp index
#define DS_FINAL_MSG_POLL_TX_TS_IDX  10 // time stamp index
#define DS_FINAL_MSG_RESP_RX_TS_IDX  14 // time stamp index
#define DS_FINAL_MSG_FINAL_TX_TS_IDX 18 // time stamp index

#define FC_BLINK 0xC5 
#define FC2_BLINK 0x00
#define FC_TWR 0x41   
#define FC2_TWR 0x88  

#define FUNCTION_CODE_BLINK 0xB1
#define FUNCTION_CODE_BLINK_ACK 0xB2
#define FUNCTION_CODE_SS_POLL 0xE0
#define FUNCTION_CODE_SS_RESP 0xE1
#define FUNCTION_CODE_DS_POLL 0x21
#define FUNCTION_CODE_DS_RESP 0x10
#define ACTIVITY_CODE_DS_RESP 0x02
#define FUNCTION_CODE_DS_FINAL 0x23

#define ALL_MSG_COMMON_LEN 10
#define POLL_MSG_LEN 12  // includes crc bytes
#define SS_RESP_MSG_LEN 20  // includes crc bytes
#define DS_RESP_MSG_LEN 15  // includes crc bytes
#define FINAL_MSG_LEN 24 // includes crc bytes

#define PAN_ID 0xCAFE

#define SNIFF_ON_TIME  2
#define SNIFF_OFF_TIME 16

#define RX_BUF_LEN 127 // Maximum message size for DW3000

#define IRQ 1

typedef enum {
    UWB_STATE_IDLE,
    UWB_STATE_INIT,
    UWB_STATE_RESPOND,
    UWB_STATE_ERROR,
    UWB_STATE_TIMEOUT,
} uwb_state_t;

typedef enum {
    UWB_XCHANGE_SS_INIT,
    UWB_XCHANGE_DS_INIT,
    UWB_XCHANGE_SS_RESP,
    UWB_XCHANGE_DS_RESP,
    UWB_XCHANGE_XX_RESP,
    UWB_XCHANGE_OTHER,
    UWB_XCHANGE_IDLE,
} uwb_xchange_type;

typedef enum {
    IDLE,
    TWR_POLL_TX,           // DS or SS, we sent the poll
    TWR_POLL_RX,           // DS or SS, we wait for poll
    TWR_SS_RESP_RX,               
    TWR_SS_RESP_TX,         
    TWR_DS_FINAL_TX,         
    TWR_DS_RESP_TX,         
    TWR_DS_FINAL_RX,         
    TWR_ABORT,              // DS or SS, we abort the exchange for some reason   
    TWR_SUCCESS_INIT,       // DS or SS, we successfully completed the exchange from the initiator pov
    TWR_SUCCESS_RESP,       // DS or SS, we successfully completed the exchange from the responder pov
} uwb_xchange_step;

const char* step2string(uwb_xchange_step step){
    switch (step) {
        case IDLE: return "IDLE";
        case TWR_POLL_TX: return "TWR_POLL_TX";
        case TWR_POLL_RX: return "TWR_POLL_RX";
        case TWR_SS_RESP_RX: return "TWR_SS_RESP_RX";
        case TWR_SS_RESP_TX: return "TWR_SS_RESP_TX";
        case TWR_DS_FINAL_TX: return "TWR_DS_FINAL_TX";
        case TWR_DS_RESP_TX: return "TWR_DS_RESP_TX";
        case TWR_DS_FINAL_RX: return "TWR_DS_FINAL_RX";
        case TWR_ABORT: return "TWR_ABORT";
        case TWR_SUCCESS_INIT: return "TWR_SUCCESS_INIT";
        case TWR_SUCCESS_RESP: return "TWR_SUCCESS_RESP";
        default: return "UNKNOWN STEP";
    }
}

const char* xchange2string(uwb_xchange_type xchange){
    switch (xchange) {
        case UWB_XCHANGE_SS_INIT: return "UWB_XCHANGE_SS_INIT";
        case UWB_XCHANGE_DS_INIT: return "UWB_XCHANGE_DS_INIT";
        case UWB_XCHANGE_SS_RESP: return "UWB_XCHANGE_SS_RESP";
        case UWB_XCHANGE_DS_RESP: return "UWB_XCHANGE_DS_RESP";
        case UWB_XCHANGE_XX_RESP: return "UWB_XCHANGE_XX_RESP";
        case UWB_XCHANGE_OTHER: return "UWB_XCHANGE_OTHER";
        case UWB_XCHANGE_IDLE: return "UWB_XCHANGE_IDLE";
        default: return "UNKNOWN XCHANGE";
    }
}

typedef struct {
    uwb_xchange_type xchange; // is this node the initiator or the responder
    uwb_xchange_step step; // which step in the TWR exchange
    time_stamps_t time_stamps;
} uwb_xsession_t;


uwb_xsession_t xsession ={
    .xchange = UWB_XCHANGE_IDLE,
    .step = IDLE,
    .time_stamps = EMPTY_STAMPS,
};

uwb_state_t uwb_state = UWB_STATE_IDLE;

static const char *TAG = "UWB_TASKS";

static uint8_t tx_msg_buffer[] ={0x41, 0x88, 0, 0xCA, 0xFE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_buffer[RX_BUF_LEN];

static uint32_t status_reg;  
static double tof, distance;
extern dwt_txconfig_t txconfig_options;
uint8_t sequence_number = 0;

//the SPI handle
spi_device_handle_t local_spi_handle;


// UWB Configuration
dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. */
    DWT_PAC8,         /* Preamble acquisition chunk size. */
    9,                /* TX preamble code. */
    9,                /* RX preamble code. */
    1,                /* 8 symbol SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length */
    DWT_PDOA_M0       /* PDOA mode off */
};

uint16_t shortaddress = 0;
uint16_t othershortaddress = 0;

TaskHandle_t uwb_main_task_handle = NULL;

static QueueHandle_t ranging_result_Q = NULL;
static QueueHandle_t ranging_request_Q = NULL;

void uwb_setup(bool start_int, QueueHandle_t Q_ranging_result, TaskHandle_t GTtask) {

    // Store the queue handle
    ranging_result_Q = Q_ranging_result;
    // Initialize SPI and DW3000
    spiBegin(HSPI_HOST, HSPI_MOSI_GPIO, HSPI_MISO_GPIO, HSPI_SCLK_GPIO, HSPI_CS_GPIO, PIN_IRQ, PIN_RST);
    spiSelect(HSPI_CS_GPIO);
    local_spi_handle = getSPIHandle();
    reset();
    vTaskDelay(pdMS_TO_TICKS(250));  // Wait for DW3000 to start

    while (!dwt_checkidlerc()) {
        ESP_LOGE(TAG, "IDLE FAILED");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    setSPIClockSpeed(false);
    dwt_softreset();
    vTaskDelay(pdMS_TO_TICKS(200));

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        for(int i =0; i<10; i++){ 
            ESP_LOGE(TAG, "INIT FAILED");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        esp_restart();
    }
    setSPIClockSpeed(true);

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config)) {
        for(int i =0; i<10; i++){
            ESP_LOGE(TAG, "CONFIG FAILED");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        esp_restart();
    }

    // set the pan id and frame filter
    // print the mac address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    shortaddress = static_cast<uint16_t>(mac[4]<<8 | mac[5]);
    ESP_LOGI(TAG, "Short Address: %X", shortaddress);
    dwt_seteui(mac); // Set the MAC address
    dwt_setaddress16(shortaddress); // Set short address as 2 last bytes of MAC address
    dwt_setpanid(PAN_ID); // Set predefined PAN ID
    // dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, 
    //                      DWT_FF_BEACON_EN |  // Accept Beacon frames
    //                      DWT_FF_DATA_EN   |  // Accept Data frames
    //                      DWT_FF_ACK_EN    |  // Accept Acknowledgment frames
    //                      DWT_FF_MAC_EN    |  // Accept MAC Command frames
    //                      DWT_FF_COORD_EN  |  // Accept Coordinator frames (for PAN coordination ) 
    //                      DWT_FF_IMPBRCAST_EN);  
    dwt_configureframefilter(DWT_FF_DISABLE, 0x0);
    
    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    
    uint32_t gpio_mode = dwt_read32bitreg(GPIO_MODE_ID);

    // Clear the MSGP1 bits (bits 5-3) for GPIO1 configuration
    // dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    gpio_mode &= ~(GPIO_MODE_MSGP1_MODE_BIT_MASK);  // Assuming GPIO_MODE_MSGP1_MODE_BIT_MASK is defined as (0x7 << 3)

    // Set the MSGP1 field to 001 to select the SFDLED output
    gpio_mode |= (0x1 << 3);

    dwt_write32bitreg(GPIO_MODE_ID, gpio_mode);
    // Assume LED_CTRL_REG is defined as the address for Sub-register 0x11:16.
    uint32_t led_ctrl = dwt_read32bitreg(LED_CTRL_ID);

    // Clear the BLINK_TIM field (bits 7-0) while preserving other bits.
    led_ctrl &= ~0xFF;

    // Set BLINK_TIM to 0x01 (14ms = 1 * 14ms).
    led_ctrl |= 0x01;

    dwt_write32bitreg(LED_CTRL_ID, led_ctrl);

    uint32_t lp_clk = dwt_read32bitreg(SEQ_CTRL_ID);
    // SEQ_CTRL_LP_CLK_DIV_BIT_OFFSET
    // Clear the LP_CLK_DIV field (bits 31-26).
    lp_clk &= ~(0x3F << 26);  // 0x3F covers 6 bits (0b111111).

    // Set LP_CLK_DIV to 2 (which gives an effective divisor of 2 << 4 = 32).
    lp_clk |= (1<< 26);

    dwt_write32bitreg(SEQ_CTRL_ID, lp_clk); // Enable low power clock

    setGTtask(GTtask);

    if (start_int) {
        start_interrupts();
    }
    // dwt_setpreambledetecttimeout(3); // 3 symbols preamble timeout

    /* Apply default antenna delay value. See NOTE 2 below. */
    // dwt_setrxantennadelay(RX_ANT_DLY);
    // dwt_settxantennadelay(TX_ANT_DLY);
    // dwt_configureframefilter(DWT_FF_DISABLE,0x0);
    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
}

void start_interrupts(){
    ESP_LOGI(TAG, "start interrupts");
    /* Register the call-backs (SPI CRC error callback is not used). *///&spi_err_cb, &spi_rdy_cb
    dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb, NULL, NULL);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_RXFTO_BIT_MASK | SYS_STATUS_RXPTO_BIT_MASK | SYS_STATUS_RXPHE_BIT_MASK  // SYS_STATUS_RXPTO_BIT_MASK: preamble detection timeout
                        | SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK | SYS_STATUS_RXSTO_BIT_MASK , //| SYS_STATUS_SPIRDY_BIT_MASK | SYS_STATUS_HI_SPIERR_BIT_MASK
        0, DWT_ENABLE_INT_ONLY); // DWT_ENABLE_INT_ONLY or DWT_ENABLE_INT

    /*Clearing the SPI ready interrupt*/
    dwt_write32bitreg( SYS_STATUS_ID, SYS_STATUS_SPIRDY_BIT_MASK);

    /* Install DW IC IRQ handler.*/
    port_set_dwic_isr(dwt_isr);
    ESP_LOGI(TAG, "irq init next");
    dw3000_gpio_irq_init();
    ESP_LOGI(TAG, "irq init done");
    ESP_LOGI(TAG, "IRQ status: %d", port_GetEXT_IRQStatus());
    // /* Set delay to turn reception on after transmission of the frame. See NOTE 3 below. */
    // dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    /* Set response frame timeout. */
    dwt_setrxtimeout(SHORT_RX_TIMEOUT_UUS);
}

TaskHandle_t create_uwb_main_task(QueueHandle_t Q_ranging_request){
    ranging_request_Q = Q_ranging_request;
    xTaskCreate(uwb_main_task, "UWB main Task", 4096, NULL, 7, &uwb_main_task_handle);
    int ret = xTaskCreate(adc_Task, "ADC Task", 4096*4, NULL, 8, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADC task");
    }
    return uwb_main_task_handle;
}

void TWR_abort(){
    xsession.step = TWR_ABORT;
    xsession.xchange = uwb_xchange_type::UWB_XCHANGE_IDLE;
    xsession.time_stamps = EMPTY_STAMPS;
    xTaskNotifyGive(uwb_main_task_handle);
}

void process_RX(const dwt_cb_data_t *cb_data ){
    uint16_t frame_len = cb_data->datalength;
    dwt_readrxdata(rx_buffer, frame_len, 0);

    auto is_twr_frame = rx_buffer[FC_IDX]==FC_TWR && rx_buffer[FC2_IDX]==FC2_TWR;
    if(!is_twr_frame) {
        TWR_abort();
        return;
    }

    // we have a TWR frame
    switch(xsession.step){
        case TWR_POLL_RX:
            if(rx_buffer[FUNCTION_CODE_IDX] == FUNCTION_CODE_SS_POLL){
                xsession.xchange = uwb_xchange_type::UWB_XCHANGE_SS_RESP;
                xsession.step = TWR_SS_RESP_TX;
                Resp_SS_POLL_RX();
            }else if (rx_buffer[FUNCTION_CODE_IDX] == FUNCTION_CODE_DS_POLL){
                xsession.xchange = uwb_xchange_type::UWB_XCHANGE_DS_RESP;
                xsession.step = TWR_DS_RESP_TX;
                Resp_DS_POLL_RX();
            } else{
                TWR_abort();
            }
            break;
        case TWR_SS_RESP_RX:
            if(rx_buffer[FUNCTION_CODE_IDX] == FUNCTION_CODE_SS_RESP){
                Init_SS_RESP_RX();
            }else{
                TWR_abort();
            }
            break;
        case TWR_DS_FINAL_TX:
            if(rx_buffer[FUNCTION_CODE_IDX] == FUNCTION_CODE_DS_RESP){
                Init_DS_RESP_RX();
            }else{
                TWR_abort();
            }
            break;
        case TWR_DS_FINAL_RX:
            // ESP_LOGI(TAG, "TWR_DS_FINAL_RX");
            if(rx_buffer[FUNCTION_CODE_IDX] == FUNCTION_CODE_DS_FINAL){
                Resp_DS_FINAL_RX();
            }else{
                TWR_abort();
            }
            break;
        default:
            TWR_abort();
            break;
    }
        
}

void process_TX(const dwt_cb_data_t *cb_data){
    // get the current session
    switch(xsession.step){
        case TWR_POLL_TX:
            if(xsession.xchange == uwb_xchange_type::UWB_XCHANGE_SS_INIT){
                xsession.step = TWR_SS_RESP_RX;
            }else if (xsession.xchange == uwb_xchange_type::UWB_XCHANGE_DS_INIT){
                xsession.step = TWR_DS_FINAL_TX;
            } else{
                TWR_abort();
            }
            xsession.time_stamps.poll_tx.MCU_time = get_capture_time();
            xsession.time_stamps.poll_tx.DW3000_time = dwt_readtxtimestamplo32();
            break;
        case TWR_DS_FINAL_TX:
            xsession.step = TWR_SUCCESS_INIT;
            xsession.time_stamps.final_tx.MCU_time = get_capture_time();
            xsession.time_stamps.final_tx.DW3000_time = dwt_readtxtimestamplo32();
            // ESP_LOGI(TAG, "DS FINAL TX");
            Push_success(true, false);
            break;
        case TWR_SS_RESP_TX:
            xsession.step = TWR_SUCCESS_RESP;
            xsession.time_stamps.resp_tx.MCU_time = get_capture_time();
            xsession.time_stamps.resp_tx.DW3000_time = dwt_readtxtimestamplo32();
            // ESP_LOGI(TAG, "SS RESP TX");
            Push_success(false, true);
            break;
        case TWR_DS_RESP_TX:
            xsession.step = TWR_DS_FINAL_RX;
            xsession.time_stamps.resp_tx.MCU_time = get_capture_time();
            xsession.time_stamps.resp_tx.DW3000_time = dwt_readtxtimestamplo32();
            // dwt_rxenable(DWT_START_RX_IMMEDIATE);
            // ESP_LOGI(TAG, "DS RESP TX");
            break;
        default:
            TWR_abort();
            break;
    }
}

int start_tx(uint8_t* buf, size_t len, int start_flag= DWT_START_TX_IMMEDIATE | DWT_START_RX_IMMEDIATE) {
    sequence_number++;
    dwt_writetxdata(len, buf, 0);
    dwt_writetxfctrl(len, 0, 1);
    return dwt_starttx(start_flag);
}

void Push_success(bool is_initiator, bool is_SS){
    xsession.xchange = uwb_xchange_type::UWB_XCHANGE_IDLE;
    if (is_initiator){
        xsession.step = TWR_SUCCESS_INIT;
    }else{
        xsession.step = TWR_SUCCESS_RESP;
    }
    xsession.time_stamps.is_initiator = is_initiator;
    xsession.time_stamps.is_SS = is_SS;
    xQueueSendToBack(ranging_result_Q, &xsession.time_stamps, 0);
    xsession.time_stamps = EMPTY_STAMPS;
    xTaskNotifyGive(uwb_main_task_handle);

}

void Init_POLL_TX(){
    dwt_setrxtimeout(LONG_RX_TIMEOUT_UUS);
    // send the first frame
    tx_msg_buffer[FC_IDX] = FC_TWR;
    tx_msg_buffer[FC2_IDX] = FC2_TWR;
    tx_msg_buffer[ALL_MSG_SN_IDX] = sequence_number++;
    tx_msg_buffer[PAN_ID_HI_IDX] = PAN_ID >> 8;
    tx_msg_buffer[PAN_ID_LO_IDX] = PAN_ID & 0xFF;
    tx_msg_buffer[DEST_ID_HI_IDX] = 0xFF;
    tx_msg_buffer[DEST_ID_LO_IDX] = 0xFF;
    tx_msg_buffer[SOURCE_ID_HI_IDX] = shortaddress >> 8;
    tx_msg_buffer[SOURCE_ID_LO_IDX] = shortaddress & 0xFF;
    if (xsession.xchange == uwb_xchange_type::UWB_XCHANGE_SS_INIT){
        tx_msg_buffer[FUNCTION_CODE_IDX] = FUNCTION_CODE_SS_POLL;
    }else if (xsession.xchange == uwb_xchange_type::UWB_XCHANGE_DS_INIT){
        tx_msg_buffer[FUNCTION_CODE_IDX] = FUNCTION_CODE_DS_POLL;
    }
    // POLL_MSG_LEN
    start_tx(tx_msg_buffer, POLL_MSG_LEN, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

void Init_SS_RESP_RX(){
    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
    // Retrieve timestamps using resp_msg_get_ts
    poll_tx_ts = dwt_readtxtimestamplo32();
    resp_rx_ts = dwt_readrxtimestamplo32();

    float clock_offset_ratio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

    resp_msg_get_ts(&rx_buffer[SS_RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
    resp_msg_get_ts(&rx_buffer[SS_RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

    // Calculate round-trip delay and distance
    int32_t rtd_init = resp_rx_ts - poll_tx_ts;
    int32_t rtd_resp = resp_tx_ts - poll_rx_ts;

    tof = ((rtd_init - rtd_resp * (1 - clock_offset_ratio)) / 2.0) * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;

    // update session timestamps
    xsession.time_stamps.poll_rx.MCU_time = get_capture_time();
    xsession.time_stamps.poll_tx.DW3000_time = poll_tx_ts;
    xsession.time_stamps.poll_rx.DW3000_time = resp_rx_ts;
    xsession.time_stamps.resp_tx.DW3000_time = resp_tx_ts;
    xsession.time_stamps.resp_rx.DW3000_time = poll_rx_ts;
    xsession.time_stamps.SN = rx_buffer[ALL_MSG_SN_IDX];
    xsession.time_stamps.other_addr = (rx_buffer[SOURCE_ID_HI_IDX] << 8) | rx_buffer[SOURCE_ID_LO_IDX];
    xsession.time_stamps.distance = distance;
    Push_success(true, true);
}



void Init_DS_RESP_RX(){
    // uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
    // uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
    uint64_t poll_tx_ts, resp_rx_ts;
    poll_tx_ts = get_tx_timestamp_u64();
    resp_rx_ts = get_rx_timestamp_u64();
    uint32_t final_tx_time = ( (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    uint64_t final_tx_ts = (((uint64_t)(((resp_rx_ts>>8)+final_tx_time) & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    tx_msg_buffer[FC_IDX] = FC_TWR;
    tx_msg_buffer[FC2_IDX] = FC2_TWR;
    tx_msg_buffer[ALL_MSG_SN_IDX] = rx_buffer[ALL_MSG_SN_IDX];
    tx_msg_buffer[PAN_ID_HI_IDX] = PAN_ID >> 8;
    tx_msg_buffer[PAN_ID_LO_IDX] = PAN_ID & 0xFF;
    tx_msg_buffer[DEST_ID_HI_IDX] = rx_buffer[SOURCE_ID_HI_IDX];
    tx_msg_buffer[DEST_ID_LO_IDX] = rx_buffer[SOURCE_ID_LO_IDX];
    tx_msg_buffer[SOURCE_ID_HI_IDX] = shortaddress >> 8;
    tx_msg_buffer[SOURCE_ID_LO_IDX] = shortaddress & 0xFF;
    tx_msg_buffer[FUNCTION_CODE_IDX] = FUNCTION_CODE_DS_FINAL;
    // Write timestamps using resp_msg_set_ts
    final_msg_set_ts(&tx_msg_buffer[DS_FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
    final_msg_set_ts(&tx_msg_buffer[DS_FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
    final_msg_set_ts(&tx_msg_buffer[DS_FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
    
    // Transmit response
    int ret = start_tx(tx_msg_buffer, FINAL_MSG_LEN, DWT_START_TX_DLY_RS);
    
    if (ret == DWT_ERROR) {
        ESP_LOGE(TAG, "Error starting TX in Init_DS_RESP_RX");
        TWR_abort();
        return;
    }
    // float clock_offset_ratio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    // ESP_LOGI(TAG, "RESP Clk a: %f", clock_offset_ratio);
    xsession.time_stamps.resp_rx.MCU_time = get_capture_time();
    xsession.time_stamps.resp_rx.DW3000_time = (uint32_t)(resp_rx_ts & 0xffffffff);
    xsession.time_stamps.poll_tx.DW3000_time = (uint32_t)(poll_tx_ts & 0xffffffff);
    xsession.time_stamps.final_tx.DW3000_time = (uint32_t)(final_tx_ts & 0xffffffff);
    xsession.time_stamps.SN = rx_buffer[ALL_MSG_SN_IDX];
    sequence_number = sequence_number > xsession.time_stamps.SN ? sequence_number : xsession.time_stamps.SN;
    xsession.time_stamps.other_addr = (rx_buffer[SOURCE_ID_HI_IDX] << 8) | rx_buffer[SOURCE_ID_LO_IDX];
}

void Resp_start_RX(){
    // setup reception
    dwt_setrxtimeout(LONG_RX_TIMEOUT_UUS);
    // dwt_setrxaftertxdelay(50);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void Resp_SS_POLL_RX(){
    uint32_t poll_rx_ts = dwt_readrxtimestamplo32();
    uint32_t resp_tx_time = ( (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    uint32_t resp_tx_ts = (((uint64_t)(((poll_rx_ts>>8)+resp_tx_time) & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    tx_msg_buffer[FC_IDX] = FC_TWR;
    tx_msg_buffer[FC2_IDX] = FC2_TWR;
    tx_msg_buffer[ALL_MSG_SN_IDX] = rx_buffer[ALL_MSG_SN_IDX];
    tx_msg_buffer[PAN_ID_HI_IDX] = PAN_ID >> 8;
    tx_msg_buffer[PAN_ID_LO_IDX] = PAN_ID & 0xFF;
    tx_msg_buffer[DEST_ID_HI_IDX] = rx_buffer[SOURCE_ID_HI_IDX];
    tx_msg_buffer[DEST_ID_LO_IDX] = rx_buffer[SOURCE_ID_LO_IDX];
    tx_msg_buffer[SOURCE_ID_HI_IDX] = shortaddress >> 8;
    tx_msg_buffer[SOURCE_ID_LO_IDX] = shortaddress & 0xFF;
    tx_msg_buffer[FUNCTION_CODE_IDX] = FUNCTION_CODE_SS_RESP;
    // Write timestamps using resp_msg_set_ts
    resp_msg_set_ts(&tx_msg_buffer[SS_RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
    resp_msg_set_ts(&tx_msg_buffer[SS_RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

    int ret = start_tx(tx_msg_buffer, SS_RESP_MSG_LEN, DWT_START_TX_DLY_RS  | DWT_RESPONSE_EXPECTED);

    if (ret == DWT_ERROR) {
        TWR_abort();
        return;
    }
    xsession.time_stamps.resp_rx.MCU_time = get_capture_time();
    xsession.time_stamps.resp_rx.DW3000_time = poll_rx_ts;
    xsession.time_stamps.SN = rx_buffer[ALL_MSG_SN_IDX];
    sequence_number = sequence_number > xsession.time_stamps.SN ? sequence_number : xsession.time_stamps.SN;
    xsession.time_stamps.other_addr = (rx_buffer[SOURCE_ID_HI_IDX] << 8) | rx_buffer[SOURCE_ID_LO_IDX];
    
}


void Resp_DS_POLL_RX(){
    // setup session
    uint32_t poll_rx_ts = dwt_readrxtimestamplo32();
    uint32_t resp_tx_time = ( (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);
    tx_msg_buffer[FC_IDX] = FC_TWR;
    tx_msg_buffer[FC2_IDX] = FC2_TWR;
    tx_msg_buffer[ALL_MSG_SN_IDX] = rx_buffer[ALL_MSG_SN_IDX];
    tx_msg_buffer[PAN_ID_HI_IDX] = PAN_ID >> 8;
    tx_msg_buffer[PAN_ID_LO_IDX] = PAN_ID & 0xFF;
    tx_msg_buffer[DEST_ID_HI_IDX] = rx_buffer[SOURCE_ID_HI_IDX];
    tx_msg_buffer[DEST_ID_LO_IDX] = rx_buffer[SOURCE_ID_LO_IDX];
    tx_msg_buffer[SOURCE_ID_HI_IDX] = shortaddress >> 8;
    tx_msg_buffer[SOURCE_ID_LO_IDX] = shortaddress & 0xFF;
    tx_msg_buffer[FUNCTION_CODE_IDX] = FUNCTION_CODE_DS_RESP;
    tx_msg_buffer[ACTIVITY_CODE_IDX] = ACTIVITY_CODE_DS_RESP;

    // dwt_setrxtimeout(LONG_RX_TIMEOUT_UUS);
    // DS_RESP_MSG_LEN
    int ret = start_tx(tx_msg_buffer, DS_RESP_MSG_LEN, DWT_START_TX_DLY_RS  | DWT_RESPONSE_EXPECTED);

    if (ret == DWT_ERROR) {
        ESP_LOGE(TAG, "Error starting TX in Resp_DS_POLL_RX");
        TWR_abort();
        return;
    }
    // float clock_offset_ratio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    // ESP_LOGI(TAG, "POLL Clk a: %f", clock_offset_ratio);
    xsession.time_stamps.poll_rx.MCU_time = get_capture_time();
    xsession.time_stamps.poll_rx.DW3000_time = poll_rx_ts;
    xsession.time_stamps.SN = rx_buffer[ALL_MSG_SN_IDX];
    sequence_number = (sequence_number > xsession.time_stamps.SN) ? sequence_number : xsession.time_stamps.SN;
    xsession.time_stamps.other_addr = (rx_buffer[SOURCE_ID_HI_IDX] << 8) | rx_buffer[SOURCE_ID_LO_IDX];
}

void Resp_DS_FINAL_RX(){
    // setup session
    uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
    uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    double Ra, Rb, Da, Db;
    int64_t tof_dtu;

    final_msg_get_ts(&rx_buffer[DS_FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    final_msg_get_ts(&rx_buffer[DS_FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
    final_msg_get_ts(&rx_buffer[DS_FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

    poll_rx_ts_32 = xsession.time_stamps.poll_rx.DW3000_time;
    // resp_tx_ts_32 = xsession.time_stamps.resp_tx.DW3000_time;
    resp_tx_ts_32 = dwt_readtxtimestamplo32();
    final_rx_ts_32 = dwt_readrxtimestamplo32();

    Ra = (double)(resp_rx_ts - poll_tx_ts);
    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
    Da = (double)(final_tx_ts - resp_rx_ts);
    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
    tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

    tof = tof_dtu * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;

    xsession.time_stamps.poll_tx.DW3000_time = poll_tx_ts;
    xsession.time_stamps.resp_rx.DW3000_time = resp_rx_ts;
    xsession.time_stamps.resp_tx.DW3000_time = resp_tx_ts_32;
    xsession.time_stamps.final_tx.DW3000_time = final_tx_ts;
    xsession.time_stamps.final_rx.DW3000_time = final_rx_ts_32;
    xsession.time_stamps.final_rx.MCU_time = get_capture_time();
    xsession.time_stamps.SN = rx_buffer[ALL_MSG_SN_IDX];
    xsession.time_stamps.other_addr = (rx_buffer[SOURCE_ID_HI_IDX] << 8) | rx_buffer[SOURCE_ID_LO_IDX];
    sequence_number = (sequence_number > xsession.time_stamps.SN) ? sequence_number : xsession.time_stamps.SN;
    xsession.time_stamps.distance = distance;
    // float clock_offset_ratio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    // ESP_LOGI(TAG, "FINAL Clk a: %f", clock_offset_ratio);
    // Resp_Push_DS_success();
    Push_success(false, false);
}

uint32_t rx_ok_cb_counter   = 0;
uint32_t rx_to_cb_counter   = 0;
uint32_t rx_err_cb_counter  = 0;
uint32_t tx_conf_cb_counter = 0;

void rx_ok_cb(const dwt_cb_data_t *cb_data){
    rx_ok_cb_counter++;
    // ESP_LOGI(TAG, "RX OK");
    process_RX(cb_data);
}

void rx_to_cb(const dwt_cb_data_t *cb_data){
    rx_to_cb_counter++;
    // ESP_LOGI(TAG, "RX TO");
    TWR_abort();
}

void rx_err_cb(const dwt_cb_data_t *cb_data){
    rx_err_cb_counter++;
    // ESP_LOGI(TAG, "RX ERR");
    TWR_abort();
}

void tx_conf_cb(const dwt_cb_data_t *cb_data){
    tx_conf_cb_counter++;
    // ESP_LOGI(TAG, "TX CONF");
    process_TX(cb_data);
}

void start_DS_TWR(){
    xsession.xchange = uwb_xchange_type::UWB_XCHANGE_DS_INIT;
    xsession.step = TWR_POLL_TX;
    Init_POLL_TX();
}
void start_SS_TWR(){
    xsession.xchange = uwb_xchange_type::UWB_XCHANGE_SS_INIT;
    xsession.step = TWR_POLL_TX;
    Init_POLL_TX();
}

void start_RX_TWR(){
    xsession.xchange = uwb_xchange_type::UWB_XCHANGE_XX_RESP;
    xsession.step = TWR_POLL_RX;
    Resp_start_RX();
}

// get a threshold for TX from the number of attempts
int thrFromAttempts(int attempts, int max_attempt_val){
    int current_delay = attempts * attempts * 10;
    if (current_delay > max_attempt_val) current_delay = max_attempt_val;
    int jitter = esp_random() % (current_delay / 2);
    // vTaskDelay(pdMS_TO_TICKS(current_delay + jitter));
    return current_delay + jitter;
}

#define ADC_BUFFER_SIZE 2048
#define ADC_SAMPLE_RATE 80000 // Hz
#define adc_mult  5
bool print_adc_result=false;

// void PrintADCResultJson(int16_t *adc_data, uint64_t *adc_ts, int num_samples, int num_ts){
//     const char * LOCAL_TAG = "ADC";
//     ESP_LOGI(LOCAL_TAG, "{");
//     ESP_LOGI(LOCAL_TAG, "\"adc_data\": [");
//     for (int i = 0; i < num_samples; i++) {
//         if (i<num_samples-1) {
//             ESP_LOGI(LOCAL_TAG, "%d,", adc_data[i]);
//         } else {
//             ESP_LOGI(LOCAL_TAG, "%d", adc_data[i]);
//         }
//         // ESP_LOGI(LOCAL_TAG, "%d,", adc_data[i]);
//     }
//     ESP_LOGI(LOCAL_TAG, "],");
//     ESP_LOGI(LOCAL_TAG, "\"adc_ts\": [");
//     for (int i = 0; i < num_ts; i++) {
//         // if (i > 0) {
//         //     ESP_LOGI(LOCAL_TAG, ",");
//         // }
//         // ESP_LOGI(LOCAL_TAG, "%llu", adc_ts[i]);
//         if (i<num_ts-1) {
//             ESP_LOGI(LOCAL_TAG, "%llu,", adc_ts[i]);
//         } else {
//             ESP_LOGI(LOCAL_TAG, "%llu", adc_ts[i]);
//         }
//     }
//     ESP_LOGI(LOCAL_TAG, "]");
//     ESP_LOGI(LOCAL_TAG, "}");

// }
void PrintADCResultJson(int16_t *adc_data, uint64_t *adc_ts, int adc_index, int ts_index, int num_samples, int num_ts) {
    static int counter = 0;
    const char * LOCAL_TAG = "ADC";
    const int CHUNK_SIZE = 10; // Number of values per log line
    char buffer[256]; // Temporary buffer for chunked printing
    counter++;
    ESP_LOGI(LOCAL_TAG, "{");
    ESP_LOGI(LOCAL_TAG, "\"counter\": %d,", counter);
    ESP_LOGI(LOCAL_TAG, "\"adc_data\": [");

    // Print ADC data in chunks, taking wrapping into account
    for (int i = 0; i < num_samples; i += CHUNK_SIZE) {
        int offset = 0;
        for (int j = 0; j < CHUNK_SIZE && (i + j) < num_samples; j++) {
            int index = (adc_index + i + j) % num_samples;
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%d%s",
                               adc_data[index], (i + j < num_samples - 1) ? ", " : "");
            if (offset >= sizeof(buffer) - 1) break; // Prevent buffer overflow
        }
        ESP_LOGI(LOCAL_TAG, "%s", buffer);
    }

    ESP_LOGI(LOCAL_TAG, "],");
    ESP_LOGI(LOCAL_TAG, "\"adc_ts\": [");

    // Print ADC timestamps in chunks, taking wrapping into account
    for (int i = 0; i < num_ts; i += CHUNK_SIZE) {
        int offset = 0;
        for (int j = 0; j < CHUNK_SIZE && (i + j) < num_ts; j++) {
            int index = (ts_index + i + j) % num_ts;
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%llu%s",
                               adc_ts[index], (i + j < num_ts - 1) ? ", " : "");
            if (offset >= sizeof(buffer) - 1) break; // Prevent buffer overflow
        }
        ESP_LOGI(LOCAL_TAG, "%s", buffer);
    }

    ESP_LOGI(LOCAL_TAG, "]");
    ESP_LOGI(LOCAL_TAG, "}");
}



void adc_Task(void *pvParameters){
    adc_continuous_handle_t adc_handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_BUFFER_SIZE*SOC_ADC_DIGI_RESULT_BYTES*4,
        .conv_frame_size = ADC_BUFFER_SIZE*SOC_ADC_DIGI_RESULT_BYTES,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    // Configure ADC continuous mode for two channels.
    adc_continuous_config_t adc_cont_config = {
        .pattern_num = 1,
        .sample_freq_hz = ADC_SAMPLE_RATE,  // ADC sampling frequency in Hz  // SOC_ADC_SAMPLE_FREQ_THRES_HIGH
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,  // Using ADC1
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    }; 
    adc_digi_pattern_config_t adc_pattern[1] = {
        {
            .atten = ADC_ATTEN_DB_2_5,
            .channel = ADC1_CHANNEL_2,
            .unit = ADC_UNIT_1,
            .bit_width = SOC_ADC_DIGI_MIN_BITWIDTH,
        },
    };
    adc_cont_config.adc_pattern = adc_pattern;
    
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_cont_config));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    
    static int16_t adc_data[ADC_BUFFER_SIZE*adc_mult];
    uint64_t adc_ts[adc_mult];
    int index = 0;
    int index_ts = 0;
    static uint8_t adc_dma_buf[ADC_BUFFER_SIZE*SOC_ADC_DIGI_RESULT_BYTES];
    uint32_t adc_bytes_read = 0;
    while(true){
        esp_err_t ret = adc_continuous_read(adc_handle, adc_dma_buf, sizeof(adc_dma_buf), &adc_bytes_read, pdMS_TO_TICKS(1000));
        if (ret == ESP_OK && adc_bytes_read > 0) {
            int num_conv = adc_bytes_read / SOC_ADC_DIGI_RESULT_BYTES;
            adc_ts[index_ts++] = esp_timer_get_time();
            if (index_ts >= adc_mult) {
                index_ts = 0;
            }
            for (int i = 0; i < num_conv; i++) {
                // Each conversion result occupies SOC_ADC_DIGI_RESULT_BYTES (likely 4 bytes for TYPE2).
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)(adc_dma_buf + i * SOC_ADC_DIGI_RESULT_BYTES);
                // uint32_t chan = p->type2.channel;
                // uint32_t data = p->type2.data;
                uint32_t data = p->val;
                // Format into 16 bits: upper 4 bits for channel, lower 12 bits for ADC data.
                adc_data[index++] = (int16_t)(data & 0xFFF); // Assuming 12-bit ADC data
                if (index >= ADC_BUFFER_SIZE*adc_mult) {
                    index = 0;
                }
            }
            if(print_adc_result){
                PrintADCResultJson(adc_data, adc_ts, index, index_ts, ADC_BUFFER_SIZE*adc_mult, adc_mult);
                print_adc_result = false;
            }
        }
    }
    vTaskDelete(NULL);
}

void uwb_main_task(void *pvParameters){
    int requested_amount=0; // decremented when a request is processed
    int attempts = 1;
    const int max_attempt_val = 1000;
    const int max_notify_delay = (LONG_RX_TIMEOUT_UUS/1000)*5; // -> X times the timeout in ms
    const int delay_after_success = (LONG_RX_TIMEOUT_UUS/(2*1000)); // after a DS success teh init side finished before the other receives the final so we wait a bit
    while(true){
        if(!xQueuePeek(ranging_request_Q, &requested_amount, portMAX_DELAY)){
            ESP_LOGE(TAG, "uwb_main_task: xQueuePeek failed");
            ESP_LOGI(TAG, "uwb_main_task: %d requested amount", requested_amount);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        // ESP_LOGI(TAG, "INSIDE requested amount: %d", requested_amount);
        do{
            if(xsession.step == TWR_ABORT){
                attempts++;
                int TX_threshold = thrFromAttempts(attempts, max_attempt_val); // get a threshold for making decision 
                int rng = esp_random() % (int)(max_attempt_val*1.5); // random number 
                if(rng > TX_threshold){ 
                    ESP_LOGI(TAG, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  CRAZY !!! <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                    start_DS_TWR();
                } else {
                    ESP_LOGI(TAG, "__________________________________ GOING CALM ________________________________");
                    start_RX_TWR();
                }
            } else {
                attempts = 1;
                requested_amount--;
                if (xsession.step == TWR_SUCCESS_INIT) {
                    start_RX_TWR();
                } else if (xsession.step == TWR_SUCCESS_RESP) {
                    vTaskDelay(pdMS_TO_TICKS(delay_after_success));
                    start_DS_TWR();
                }
            }
            bool notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(max_notify_delay));
            if(!notif){
                ESP_LOGE(TAG, "uwb_main_task: ulTaskNotifyTake failed");
                ESP_LOGE(TAG, "current xchange: %s", xchange2string(xsession.xchange));
                ESP_LOGE(TAG, "current step: %s", step2string(xsession.step));
                ESP_LOGE(TAG, "IRQ status: %d", port_GetEXT_IRQStatus());
                log_uwberrors();
                attempts++;
                xsession.xchange = uwb_xchange_type::UWB_XCHANGE_IDLE;
                xsession.step = TWR_ABORT;
                xsession.time_stamps = EMPTY_STAMPS;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }while( (requested_amount>0) && (attempts<100)); //portMAX_DELAY
        print_adc_result = true;
        // actually remove the item from the queue //! dirty
        xQueueReceive(ranging_request_Q, &requested_amount, 0);
        requested_amount=0;
        attempts=1;
    }
}

void log_uwberrors(){
    static uint32_t errors[32] = {0};

    // memset(errors, 0, sizeof(errors));
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    // aquire_bus();
    check_for_status_errors(status_reg, errors);
    if (errors[CRC_ERR_IDX]) {
        ESP_LOGW(TAG, "CRC Error: %" PRIu32, errors[CRC_ERR_IDX]);
    }
    if (errors[RSE_ERR_IDX]) {
        ESP_LOGW(TAG, "RSE_ERR_IDX: %" PRIu32, errors[RSE_ERR_IDX]);
    }
    if (errors[PHE_ERR_IDX]) {
        ESP_LOGW(TAG, "PHE_ERR_IDX: %" PRIu32, errors[PHE_ERR_IDX]);
    }
    if (errors[SFDTO_ERR_IDX]) {
        ESP_LOGW(TAG, "SFDTO_ERR_IDX: %" PRIu32, errors[SFDTO_ERR_IDX]);
    }
    if (errors[PTO_ERR_IDX]) {
        ESP_LOGW(TAG, "PTO_ERR_IDX: %" PRIu32, errors[PTO_ERR_IDX]);
    }
    if (errors[RTO_ERR_IDX]) {
        ESP_LOGW(TAG, "RTO_ERR_IDX: %" PRIu32, errors[RTO_ERR_IDX]);
    }
    if (errors[SPICRC_ERR_IDX]) {
        ESP_LOGW(TAG, "SPICRC_ERR_IDX: %" PRIu32, errors[SPICRC_ERR_IDX]);
    }
    if (errors[TXTO_ERR_IDX]) {
        ESP_LOGW(TAG, "TXTO_ERR_IDX: %" PRIu32, errors[TXTO_ERR_IDX]);
    }
    if (errors[ARFE_ERR_IDX]) {
        ESP_LOGW(TAG, "ARFE_ERR_IDX: %" PRIu32, errors[ARFE_ERR_IDX]);
    }
    if (errors[TS_MISMATCH_ERR_IDX]) {
        ESP_LOGW(TAG, "TS_MISMATCH_ERR_IDX: %" PRIu32, errors[TS_MISMATCH_ERR_IDX]);
    }
    if (errors[BAD_FRAME_ERR_IDX]) {
        ESP_LOGW(TAG, "BAD_FRAME_ERR_IDX: %" PRIu32, errors[BAD_FRAME_ERR_IDX]);
    }
    if (errors[PREAMBLE_COUNT_ERR_IDX]) {
        ESP_LOGW(TAG, "PREAMBLE_COUNT_ERR_IDX: %" PRIu32, errors[PREAMBLE_COUNT_ERR_IDX]);
    }
    if (errors[CP_QUAL_ERR_IDX]) {
        ESP_LOGW(TAG, "CP_QUAL_ERR_IDX: %" PRIu32, errors[CP_QUAL_ERR_IDX]);
    }
    if (errors[STS_PREAMBLE_ERR]) {
        ESP_LOGW(TAG, "STS_PREAMBLE_ERR: %" PRIu32, errors[STS_PREAMBLE_ERR]);
    }
    if (errors[STS_PEAK_GROWTH_RATE_ERR]) {
        ESP_LOGW(TAG, "STS_PEAK_GROWTH_RATE_ERR: %" PRIu32, errors[STS_PEAK_GROWTH_RATE_ERR]);
    }
    if (errors[STS_ADC_COUNT_ERR]) {
        ESP_LOGW(TAG, "STS_ADC_COUNT_ERR: %" PRIu32, errors[STS_ADC_COUNT_ERR]);
    }
    if (errors[STS_SFD_COUNT_ERR]) {
        ESP_LOGW(TAG, "STS_SFD_COUNT_ERR: %" PRIu32, errors[STS_SFD_COUNT_ERR]);
    }
    if (errors[STS_LATE_FIRST_PATH_ERR]) {
        ESP_LOGW(TAG, "STS_LATE_FIRST_PATH_ERR: %" PRIu32, errors[STS_LATE_FIRST_PATH_ERR]);
    }
    if (errors[STS_LATE_COARSE_EST_ERR]) {
        ESP_LOGW(TAG, "STS_LATE_COARSE_EST_ERR: %" PRIu32, errors[STS_LATE_COARSE_EST_ERR]);
    }
    if (errors[STS_COARSE_EST_EMPTY_ERR]) {
        ESP_LOGW(TAG, "STS_COARSE_EST_EMPTY_ERR: %" PRIu32, errors[STS_COARSE_EST_EMPTY_ERR]);
    }
    if (errors[STS_HIGH_NOISE_THREASH_ERR]) {
        ESP_LOGW(TAG, "STS_HIGH_NOISE_THREASH_ERR: %" PRIu32, errors[STS_HIGH_NOISE_THREASH_ERR]);
    }
    if (errors[STS_NON_TRIANGLE_ERR]) {
        ESP_LOGW(TAG, "STS_NON_TRIANGLE_ERR: %" PRIu32, errors[STS_NON_TRIANGLE_ERR]);
    }
    if (errors[STS_LOG_REG_FAILED_ERR]) {
        ESP_LOGW(TAG, "STS_LOG_REG_FAILED_ERR: %" PRIu32, errors[STS_LOG_REG_FAILED_ERR]);
    }
}