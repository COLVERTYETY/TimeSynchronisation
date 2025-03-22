#ifndef LORA_TASKS_H
#define LORA_TASKS_H
#include <tinysync.h>
#include "messages.pb.h"
// #include "LoraMesher.h"

// void lora_setup();
void twrts_decode_push(Twrts twrts);
void lora_setup(QueueHandle_t receive_lora_Q, QueueHandle_t emit_lora_Q, QueueHandle_t ranging_request_Q);
void loraReceiveTask(void* pvParameters);
void log_info();

#endif  // LORA_TASKS_H
