// #include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "freertos/task_snapshot.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "uwb_tasks.h"
#include "lora_tasks.h"
#include "tinysync.h"
#include "freertos/queue.h"
// #include "esp_app_trace.h"
// #include "esp_sysview_trace.h"
#include "conf.h"

static const char *TAG = "MAIN_APP";

QueueHandle_t ranging_request_Q;  // an int that indicates the number of wanted successful requests
QueueHandle_t ranging_result_Q;   // the struct with the results
QueueHandle_t emit_lora_Q;        // the struct with the results
QueueHandle_t receive_lora_Q;     // the struct with the results

extern "C" void app_main(void) {
    
    ESP_LOGI(TAG, "Starting UWB Initialization...");

    // short delayed start
    for(int i=20; i>0; i--){
        ESP_LOGI(TAG, "Starting in %d seconds...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // create the queue
    ranging_request_Q = xQueueCreate(RANGING_REQUEST_QUEUE_LENGTH, sizeof(int));
    ranging_result_Q  = xQueueCreate(RANGING_RESULT_QUEUE_LENGTH, sizeof(time_stamps_t));
    emit_lora_Q       = xQueueCreate(EMIT_LORA_QUEUE_LENGTH, sizeof(time_stamps_t));
    receive_lora_Q    = xQueueCreate(RECEIVE_LORA_QUEUE_LENGTH, sizeof(time_stamps_t));

    configASSERT(ranging_request_Q);
    configASSERT(ranging_result_Q);
    configASSERT(emit_lora_Q);
    configASSERT(receive_lora_Q);

    TaskHandle_t GTtask = create_sync_task(ranging_result_Q, ranging_request_Q, emit_lora_Q, receive_lora_Q);
    create_uwb_main_task(ranging_request_Q);
    uwb_setup(true, ranging_result_Q, GTtask);
    // start_interrupts();
    lora_setup(receive_lora_Q, emit_lora_Q, ranging_request_Q);


    vTaskDelete(NULL);
}
