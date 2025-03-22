#ifndef UWB_TASKS_H
#define UWB_TASKS_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dw3000.h"


/**
 * @brief Setup spi and init the dw ic 
 * 
 * @param Q_ranging_result The queue handle for sending ranging results, this task is a consumer
 * @return void
 */
void uwb_setup(bool start_int, QueueHandle_t Q_ranging_result, TaskHandle_t GTtask);
// void uwb_setup(bool start_interrupts, QueueHandle_t Q_ranging_result);

void adc_Task(void *pvParameters);

// register the interrupts
void start_interrupts();

/**
 * @brief Create the main UWB task.
 * 
 * @param Q_ranging_request The queue handle for sending ranging requests
 * @return TaskHandle_t The handle of the created task.
 */
TaskHandle_t create_uwb_main_task(QueueHandle_t Q_ranging_request);

// Main task, handle session xchange and establish link  
void uwb_main_task(void *pvParameters);

// utils
void log_uwberrors();

//handle the received frame
void process_RX(const dwt_cb_data_t *cb_data);
// handle the transmitted frame
void process_TX(const dwt_cb_data_t *cb_data);

/* TWR */ 

//DS or SS, start RX
void Resp_start_RX();       
//SS, handle recption of poll and send response
void Resp_SS_POLL_RX();     
//DS, handle reception of poll and send response with timestamps
void Resp_DS_POLL_RX();     
//DS, handle reception of final
void Resp_DS_FINAL_RX();    

//DS or SS, send poll
void Init_POLL_TX();
//SS, handle reception of response       
void Init_SS_RESP_RX();
//DS, handle reception of response and send final with timestamps     
void Init_DS_RESP_RX();     


// abort TWR
void TWR_abort();

// update session, push timestamps to Q and notify main task
void Push_success(bool is_initiator, bool is_SS);

// write buffer to device and start transmission
int start_tx(uint8_t* buf, size_t len, int start_flag);


/* Declaration of cb functions. */
void rx_ok_cb(const dwt_cb_data_t *cb_data);
void rx_to_cb(const dwt_cb_data_t *cb_data);
void rx_err_cb(const dwt_cb_data_t *cb_data);
void tx_conf_cb(const dwt_cb_data_t *cb_data);

#endif  // UWB_TASKS_H
