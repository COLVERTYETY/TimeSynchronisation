#ifndef TINYSYNC_H
#define TINYSYNC_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))


// time sync 
typedef struct {
    uint64_t MCU_time; //   MCU time stamp, only available on poll_tx and poll_rx
    uint64_t DW3000_time; // dw3000 internal time stamps from TWR
} time_pair_t;

#define EMPTY_PAIR (time_pair_t){.MCU_time = 0, .DW3000_time = 0}

typedef struct {
    double mcu; // the tx event timestamp    
    double uwb; // the rx event timestamp
} time_us_t;

#define EMPTY_PAIR_US (time_us_t){.mcu = 0.0, .uwb = 0.0}

typedef struct {
    time_us_t tx; // the tx event timestamp    
    // time_us_t adjusted_tx; // the tx event timestamp    
    time_us_t rx; // the rx event timestamp
    time_us_t adjusted_rx; // the rx event timestamp
    double tof_dtu; // time of flight
    bool is_init;  // is it from our side?
    bool is_SS;     // is it a SS event?
} event_pair_t;

#define EMPTY_EVENT (event_pair_t){.tx = EMPTY_PAIR_US, .rx = EMPTY_PAIR_US, .tof_dtu = 0.0, .is_init = false, .is_SS = false}

typedef struct {
    time_pair_t poll_tx;  //  emit -->
    time_pair_t poll_rx;  //                 |receive
    time_pair_t resp_tx;  //                 <-- emit
    time_pair_t resp_rx;  //  receive|
    time_pair_t final_tx; //  emit -->
    time_pair_t final_rx; //                 |receive  
    uint8_t SN;           // sequence number
    uint16_t other_addr;
    double distance;
    bool is_initiator;
    bool is_SS;
} time_stamps_t;

#define EMPTY_STAMPS (time_stamps_t){ \
    .poll_tx = EMPTY_PAIR, \
    .poll_rx = EMPTY_PAIR, \
    .resp_tx = EMPTY_PAIR, \
    .resp_rx = EMPTY_PAIR, \
    .final_tx = EMPTY_PAIR, \
    .final_rx = EMPTY_PAIR, \
    .SN = 0, \
    .other_addr = 0, \
    .distance = 0.0, \
    .is_initiator = false, \
    .is_SS = false \
}

// Structure to hold the results.
struct MixedEffectsResults {
    double beta0;         // Fixed-effect intercept
    double beta1;         // Fixed-effect slope
    double sigma2;        // Residual variance
    double sigma_u2;      // Random-intercept variance
    double u_hat[2];      // Estimated group offsets for 2 groups
    int iterations;
};

#define EMPTY_MIXED_EFFECTS (struct MixedEffectsResults){ \
    .beta0 = 0.0, \
    .beta1 = 0.0, \
    .sigma2 = -1.0, \
    .sigma_u2 = -1.0, \
    .u_hat = {0.0, 0.0}, \
    .iterations = 0 \
}

typedef struct {
    double mcu_ts;
    uint32_t counter;
    MixedEffectsResults MixedResults;
    double slope;
    double intercept;
    double std_err;
    float temperature;
} GT_event_t;

#define EMPTY_GT_EVENT (GT_event_t){ \
    .mcu_ts = 0.0, \
    .counter = 0, \
    .MixedResults = EMPTY_MIXED_EFFECTS, \
    .slope = 0.0, \
    .intercept = 0.0, \
    .std_err = 0.0 \
}

void printTimestamps(time_stamps_t twr);
void print_buffer(time_stamps_t *buffer, int buffer_size);

void LinearRegression(event_pair_t *event_buffer, int buffer_size, double *slope, double *intercept, double *std_err, bool (*filter)(const event_pair_t&, double&, double&));
double computeToFDS(time_stamps_t twr);


time_us_t pair2us(time_pair_t pair);
event_pair_t createPair(bool is_init, bool is_SS, double tof, time_pair_t tx, time_pair_t rx);
void Ts2events(time_stamps_t *twr, event_pair_t *event_buffer, int *buffer_index,const int buffer_size);
void addSyncData(time_stamps_t *twr);
double mcu_to_us(uint64_t mcu_ticks);
double uwb_to_us(int64_t diff_in_ticks);
int find_matching(time_stamps_t reference, time_stamps_t *buffer, int buffer_size);

void coordinator_decision(bool our_side, time_stamps_t current, time_stamps_t* storage_buffer, time_stamps_t* compare_buffer, int *storage_index, int buffer_size);
void periodic_request_Task(void *pvParameters);
void sync_coordinator_Task(void *pvParameters);

/**
 * @brief Creates the Tiny Sync task that processes messages from the provided queues.
 * 
 * The task consumes messages from the ranging result queue, processes the ranging results,
 * and sends messages to the specified LoRa and ranging request queues.
 * 
 * @param Q_ranging_result The handle to the queue from which the task will consume ranging result messages.
 * @param Q_ranging_request The handle to the queue for ranging requests.
 * @param Q_emit_lora The handle to the queue to which the task will send LoRa messages.
 * @param Q_receive_lora The handle to the queue to which the task will send processed messages.
 * @return TaskHandle_t The handle to the created task.
 */
TaskHandle_t create_sync_task(QueueHandle_t Q_ranging_result, QueueHandle_t Q_ranging_request, QueueHandle_t Q_emit_lora,  QueueHandle_t Q_receive_lora);

#endif