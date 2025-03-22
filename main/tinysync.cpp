#include "tinysync.h"
#include <cstring>
#include "esp_log.h"
#include "conf.h"
#include "esp_timer.h"
#include "driver/temperature_sensor.h"
#include <functional>
#include <cmath>
#include "dw3000_port.h"


static QueueHandle_t ranging_result_Q;
static QueueHandle_t ranging_request_Q;
static QueueHandle_t emit_lora_Q;
static QueueHandle_t receive_lora_Q;

static const char *TAG = "SYNC";



void LinearRegression(event_pair_t *event_buffer, int buffer_size, double *slope, double *intercept, double *std_err, bool (*filter)(const event_pair_t&, double&, double&)) {
    // Calculate the slope and intercept using linear regression
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
    int count = 0;

    for (int i = 0; i < buffer_size; ++i) {
        double x, y;
        if (filter(event_buffer[i], x, y)) {
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_xx += x * x;
            count++;
        }
    }

    if (count < 2) {
        *slope = 0.0;
        *intercept = 0.0;
        *std_err = -1.0;  // Error indicator
        return;
    }

    double x_mean = sum_x / count;
    double y_mean = sum_y / count;

    double numerator = sum_xy - count * x_mean * y_mean;
    double denominator = sum_xx - count * x_mean * x_mean;

    if (fabs(denominator) < 1e-9) {
        *slope = 0.0;
        *intercept = 0.0;
        *std_err = -1.0;
        return;
    }

    *slope = numerator / denominator;
    *intercept = y_mean - (*slope) * x_mean;

    if(std_err == nullptr) return;
    // Compute standard error
    double sum_err = 0.0;
    for (int i = 0; i < buffer_size; ++i) {
        double x, y;
        if (filter(event_buffer[i], x, y)) {
            double predicted_y = (*slope) * x + (*intercept);
            sum_err += (y - predicted_y) * (y - predicted_y);
        }
    }

    *std_err = sqrt(sum_err / count);
}


//  combinedFilter: Determines whether the event is valid.
//  It also computes the x and y values as follows:
//    - If event.is_init is true:
//         x = event.tx.mcu, y = event.adjusted_rx.mcu, group = 0.
//    - Otherwise:
//         x = event.adjusted_rx.mcu, y = event.tx.mcu, group = 1.
//  It also applies common quality checks.
bool combinedFilter(const event_pair_t &event, double &x, double &y, int &group) {
    // Common filtering conditions.
    if ((event.tx.mcu == 0.0 && event.rx.mcu == 0.0) ||
        (std::fabs(event.tof_dtu) > 2.0)) {
        return false;
    }
    // For group 0 (e.g., restricted sampling method)
    if (event.is_init) {
        x = event.tx.mcu;
        y = event.adjusted_rx.mcu;
        group = 0;
        return true;
    }
    else {
        // For group 1 (e.g., inverted sampling method)
        x = event.adjusted_rx.mcu;
        y = event.tx.mcu;
        group = 1;
        return true;
    }
}

//----------------------------------------------------------------------
// This function implements a simple mixed-effects regression using an EM
// algorithm. It uses fixed-size arrays for the 2 groups and reuses previous
// estimates (stored in static variables) to help warm-start successive calls.
// It loops over the provided event_buffer and uses combinedFilter() to extract
// valid data points (with their x, y values and group id).
MixedEffectsResults fit_mixed_effects_EM(const event_pair_t *event_buffer, int buffer_size) {
    // Static state to hold previous estimates across calls.
    static bool first_call = true;
    static double prev_u_hat[2] = {0.0, 0.0}; // group offsets
    static double prev_v[2]     = {1.0, 1.0}; // uncertainty in offsets

    // ------------------------------------------------------------------
    // First pass: Compute overall OLS estimates ignoring group effects.
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
    int total_count = 0;
    // Also count per group for initial offset estimates.
    int group_count[2] = {0, 0};

    for (int i = 0; i < buffer_size; ++i) {
        double x, y;
        int group;
        if (combinedFilter(event_buffer[i], x, y, group)) {
            sum_x   += x;
            sum_y   += y;
            sum_xy  += x * y;
            sum_xx  += x * x;
            total_count++;
            group_count[group]++;
        }
    }
    if (total_count < 2) {
        MixedEffectsResults res;
        res.beta0 = 0.0;
        res.beta1 = 0.0;
        res.sigma2 = -1.0;
        res.sigma_u2 = -1.0;
        res.iterations = 0;
        res.u_hat[0] = (first_call ? 0.0 : prev_u_hat[0]);
        res.u_hat[1] = (first_call ? 0.0 : prev_u_hat[1]);
        return res;
    }
    double x_mean = sum_x / total_count;
    double y_mean = sum_y / total_count;
    double numerator = sum_xy - total_count * x_mean * y_mean;
    double denominator = sum_xx - total_count * x_mean * x_mean;
    if (std::fabs(denominator) < 1e-9) {
        MixedEffectsResults res;
        res.beta0 = 0.0;
        res.beta1 = 0.0;
        res.sigma2 = -1.0;
        res.sigma_u2 = -1.0;
        res.iterations = 0;
        res.u_hat[0] = (first_call ? 0.0 : prev_u_hat[0]);
        res.u_hat[1] = (first_call ? 0.0 : prev_u_hat[1]);
        return res;
    }
    double beta1 = numerator / denominator;
    double beta0 = y_mean - beta1 * x_mean;

    // Compute residual error (for sigma2) over all valid points.
    double sum_err = 0.0;
    for (int i = 0; i < buffer_size; ++i) {
        double x, y;
        int group;
        if (combinedFilter(event_buffer[i], x, y, group)) {
            double err = y - (beta0 + beta1 * x);
            sum_err += err * err;
        }
    }
    double sigma2 = sum_err / total_count;

    // For initial group offsets, compute average residual per group.
    double group_res_sum[2] = {0.0, 0.0};
    for (int i = 0; i < buffer_size; ++i) {
        double x, y;
        int group;
        if (combinedFilter(event_buffer[i], x, y, group)) {
            double err = y - (beta0 + beta1 * x);
            group_res_sum[group] += err;
        }
    }
    double group_res_avg[2] = {0.0, 0.0};
    for (int g = 0; g < 2; g++) {
        if (group_count[g] > 0)
            group_res_avg[g] = group_res_sum[g] / group_count[g];
        else
            group_res_avg[g] = 0.0;
    }
    // Initial sigma_u2 as the average squared offset.
    double sigma_u2 = 0.0;
    int groups_used = 0;
    for (int g = 0; g < 2; g++) {
        if (group_count[g] > 0) {
            sigma_u2 += group_res_avg[g] * group_res_avg[g];
            groups_used++;
        }
    }
    if (groups_used > 0)
        sigma_u2 /= groups_used;
    else
        sigma_u2 = 1.0;

    // Initialize u_hat and v from previous estimates (if available).
    double u_hat[2], v[2];
    if (!first_call) {
        u_hat[0] = prev_u_hat[0];
        u_hat[1] = prev_u_hat[1];
        v[0] = prev_v[0];
        v[1] = prev_v[1];
    } else {
        u_hat[0] = group_res_avg[0];
        u_hat[1] = group_res_avg[1];
        v[0] = 1.0;
        v[1] = 1.0;
    }

    // ------------------------------------------------------------------
    // EM iterations
    const int max_iter = 1000; // maximum iterations
    const double tol = 1e-9;
    int iter = 0;
    double change = tol + 1.0;
    while (iter < max_iter && change > tol) {
        double beta0_old = beta0, beta1_old = beta1, sigma2_old = sigma2, sigma_u2_old = sigma_u2;

        // E-step: For each group, compute sum of residuals and counts.
        int group_count_iter[2] = {0, 0};
        double group_sum_iter[2] = {0.0, 0.0};
        for (int i = 0; i < buffer_size; i++) {
            double x, y;
            int group;
            if (combinedFilter(event_buffer[i], x, y, group)) {
                group_count_iter[group]++;
                group_sum_iter[group] += (y - beta0 - beta1 * x);
            }
        }
        // Update group offsets u_hat and their variances v.
        for (int g = 0; g < 2; g++) {
            if (group_count_iter[g] > 0) {
                v[g] = 1.0 / (1.0 / sigma_u2 + group_count_iter[g] / sigma2);
                u_hat[g] = v[g] * (group_sum_iter[g] / sigma2);
            }
            // If no points in this group, u_hat and v remain unchanged.
        }

        // M-step: Recompute fixed effects (beta0, beta1) from adjusted response.
        double sum_x_adj = 0.0, sum_y_adj = 0.0, sum_xy_adj = 0.0, sum_xx_adj = 0.0;
        int total_count_adj = 0;
        for (int i = 0; i < buffer_size; i++) {
            double x, y;
            int group;
            if (combinedFilter(event_buffer[i], x, y, group)) {
                double y_adj = y - u_hat[group];
                sum_x_adj += x;
                sum_y_adj += y_adj;
                sum_xy_adj += x * y_adj;
                sum_xx_adj += x * x;
                total_count_adj++;
            }
        }
        if (total_count_adj < 2)
            break;  // Not enough data points

        double x_mean_adj = sum_x_adj / total_count_adj;
        double y_mean_adj = sum_y_adj / total_count_adj;
        double num_adj = sum_xy_adj - total_count_adj * x_mean_adj * y_mean_adj;
        double den_adj = sum_xx_adj - total_count_adj * x_mean_adj * x_mean_adj;
        if (std::fabs(den_adj) < 1e-9)
            break;
        beta1 = num_adj / den_adj;
        beta0 = y_mean_adj - beta1 * x_mean_adj;

        // Update sigma2 (residual variance).
        double sum_sq = 0.0;
        int group_count2[2] = {0, 0};
        for (int i = 0; i < buffer_size; i++) {
            double x, y;
            int group;
            if (combinedFilter(event_buffer[i], x, y, group)) {
                double res = y - beta0 - beta1 * x - u_hat[group];
                sum_sq += res * res;
                group_count2[group]++;
            }
        }
        // Add uncertainty from the group estimates.
        for (int g = 0; g < 2; g++) {
            sum_sq += group_count2[g] * v[g];
        }
        sigma2 = sum_sq / total_count_adj;

        // Update sigma_u2 as the average of (u_hat^2 + v) over groups that had data.
        double sum_group = 0.0;
        int groups_used2 = 0;
        for (int g = 0; g < 2; g++) {
            if (group_count2[g] > 0) {
                sum_group += (u_hat[g] * u_hat[g] + v[g]);
                groups_used2++;
            }
        }
        if (groups_used2 > 0)
            sigma_u2 = sum_group / groups_used2;

        change = std::fabs(beta0 - beta0_old) + std::fabs(beta1 - beta1_old)
                 + std::fabs(sigma2 - sigma2_old) + std::fabs(sigma_u2 - sigma_u2_old);
        iter++;
    }

    // Save current u_hat and v for next call.
    prev_u_hat[0] = u_hat[0];
    prev_u_hat[1] = u_hat[1];
    prev_v[0] = v[0];
    prev_v[1] = v[1];
    first_call = false;

    MixedEffectsResults results;
    results.beta0 = beta0;
    results.beta1 = beta1;
    results.sigma2 = sigma2;
    results.sigma_u2 = sigma_u2;
    results.iterations = iter;
    results.u_hat[0] = u_hat[0];
    results.u_hat[1] = u_hat[1];
    return results;
}

void printMixedEffectsResults(const MixedEffectsResults &results) {
    static int counter = 0;
    ESP_LOGI("MRJSON", "{");
    ESP_LOGI("MRJSON", "\"counter\": %d,", counter++);
    ESP_LOGI("MRJSON", "\"timer\": %" PRIu64 ",", esp_timer_get_time());
    ESP_LOGI("MRJSON", "\"intercept\": %.17g,", results.beta0);
    ESP_LOGI("MRJSON", "\"slope\": %.17g,", results.beta1);
    ESP_LOGI("MRJSON", "\"residual_variance\": %.17g,", results.sigma2);
    ESP_LOGI("MRJSON", "\"random_variance\": %.17g,", results.sigma_u2);
    ESP_LOGI("MRJSON", "\"iterations\": %d,", results.iterations);
    ESP_LOGI("MRJSON", "\"offset_group_0\": %.17g,", results.u_hat[0]);
    ESP_LOGI("MRJSON", "\"offset_group_1\": %.17g", results.u_hat[1]);
    ESP_LOGI("MRJSON", "}");
}

bool filterFunction(const event_pair_t &event, double &x, double &y) {
    if (event.is_init) {
        x = event.tx.mcu;
        y = event.rx.mcu;
        return true;
    }
    return false;
}

bool filterFunctionDS(const event_pair_t &event, double &x, double &y) {
    if(event.tx.mcu == 0 && event.rx.mcu == 0) return false;

    if(event.is_init) { // us as x and other as y
        x = event.tx.mcu;
        // y = event.rx.mcu;
        // x = event.adjusted_tx.mcu;
        y = event.adjusted_rx.mcu;
    } else {
        // x = event.rx.mcu;
        y = event.tx.mcu;
        x = event.adjusted_rx.mcu;
        // y = event.adjusted_tx.mcu;
    }
    return true;
}


// sampling method 1
bool restrictedFilterFunctionDS(const event_pair_t &event, double &x, double &y) {
    if( (event.tx.mcu == 0 && event.rx.mcu == 0) || (!event.is_init) || fabs(event.tof_dtu)>2.0) return false;

    if(event.is_init) { // us as x and other as y
        x = event.tx.mcu;
        y = event.adjusted_rx.mcu;
    }
    return true;
}

// sampling method 2
bool invertedrestrictedFilterFunctionDS(const event_pair_t &event, double &x, double &y) {
    if( (event.tx.mcu == 0 && event.rx.mcu == 0) || (event.is_init) || fabs(event.tof_dtu)>2.0  ) return false;

    if(!event.is_init) { // us as x and other as y
        y = event.tx.mcu;
        x = event.adjusted_rx.mcu;
    }
    return true;
}


// filter function for rx uwb to rx mcu only our side
bool filterFunctionrxA(const event_pair_t &event, double &x, double &y) {
    if(event.tx.mcu == 0 && event.rx.mcu == 0) return false;

    if(!event.is_init) {
        return false;
    }

    x = event.rx.uwb;
    y = event.rx.mcu;

    return true;
}

// filter function for rx uwb to rx mcu only other side
bool filterFunctionrxB(const event_pair_t &event, double &x, double &y) {
    if(event.tx.mcu == 0 && event.rx.mcu == 0) return false;

    if(event.is_init) {
        return false;
    }

    x = event.rx.uwb;
    y = event.rx.mcu;

    return true;
}

// filter function for tx uwb to tx mcu only our side
bool filterFunctiontxA(const event_pair_t &event, double &x, double &y) {
    if(event.tx.mcu == 0 && event.rx.mcu == 0) return false;

    if(!event.is_init) {
        return false;
    }

    x = event.tx.uwb;
    y = event.tx.mcu;

    return true;
}

// filter function for tx uwb to tx mcu only other side
bool filterFunctiontxB(const event_pair_t &event, double &x, double &y) {
    if(event.tx.mcu == 0 && event.rx.mcu == 0) return false;

    if(event.is_init) {
        return false;
    }

    x = event.tx.uwb;
    y = event.tx.mcu;

    return true;
}

double computeToFDS(time_stamps_t twr){
    double Ra = (double)twr.resp_rx.DW3000_time - (double)twr.poll_tx.DW3000_time;
    double Rb = (double)twr.final_rx.DW3000_time - (double)twr.resp_tx.DW3000_time;
    double Da = (double)twr.final_tx.DW3000_time - (double)twr.resp_rx.DW3000_time;
    double Db = (double)twr.resp_tx.DW3000_time - (double)twr.poll_rx.DW3000_time;
    double tof_dtu = ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
    const double TICK_PERIOD_US = (1.0/499.2/128.0); //!< = 15.65e-6 us
    return tof_dtu*TICK_PERIOD_US; // in us
}

time_us_t pair2us(time_pair_t pair){
    time_us_t us_pair;
    us_pair.mcu = mcu_to_us(pair.MCU_time);
    us_pair.uwb = uwb_to_us(pair.DW3000_time);
    return us_pair;
}

event_pair_t createPair(bool is_init, bool is_SS, double tof, time_pair_t tx, time_pair_t rx){
    time_us_t tx_us = pair2us(tx);
    time_us_t rx_us = pair2us(rx);
    time_us_t adjusted_rx_us = rx_us;
    adjusted_rx_us.uwb -= tof;
    adjusted_rx_us.mcu -= tof;
    event_pair_t pair = {tx_us, rx_us, adjusted_rx_us, tof, is_init, is_SS};
    return pair;
}

void Ts2events(time_stamps_t *twr, event_pair_t *event_buffer, int *buffer_index, const int buffer_size){
    // 1 twr -> 3 pairs -> 6 events -> 12 timestamps
    // compute tof
    double tof_dtu = computeToFDS(*twr);
    bool is_init = twr->is_initiator;
    bool is_SS = twr->is_SS;
    // create pairs
    *buffer_index = (*buffer_index+1) % buffer_size;
    event_buffer[*buffer_index] = createPair(  is_init, is_SS, tof_dtu, twr->poll_tx, twr->poll_rx);
    *buffer_index = (*buffer_index+1) % buffer_size;
    event_buffer[*buffer_index] = createPair( !is_init, is_SS, tof_dtu, twr->resp_tx, twr->resp_rx);
    *buffer_index = (*buffer_index+1) % buffer_size;
    event_buffer[*buffer_index] = createPair(  is_init, is_SS, tof_dtu, twr->final_tx, twr->final_rx);
}

void printEventBuffer(event_pair_t *event_buffer, int buffer_size){
    ESP_LOGI(TAG, "Printing event buffer");
    for(int i = 0; i < buffer_size; i++){
        ESP_LOGI(TAG, "Event %d", i);
        ESP_LOGI(TAG, "TX: MCU: %3.2f us, UWB: %3.2f us", event_buffer[i].tx.mcu, event_buffer[i].tx.uwb);
        ESP_LOGI(TAG, "RX: MCU: %3.2f us, UWB: %3.2f us", event_buffer[i].rx.mcu, event_buffer[i].rx.uwb);
        ESP_LOGI(TAG, "Adjusted RX: MCU: %3.2f us, UWB: %3.2f us", event_buffer[i].adjusted_rx.mcu, event_buffer[i].adjusted_rx.uwb);
        ESP_LOGI(TAG, "ToF: %3.2f us", event_buffer[i].tof_dtu);
        ESP_LOGI(TAG, "Is Init: %d, Is SS: %d", event_buffer[i].is_init, event_buffer[i].is_SS);
    }
    ESP_LOGI(TAG, "End of event buffer");
}

void printEventBufferJson(event_pair_t *event_buffer, int buffer_size, double slope, double intercept, double std_err){
    ESP_LOGI(TAG, "Printing event buffer");
    ESP_LOGI(TAG, "{");
    ESP_LOGI(TAG, "\"index\": %d,", buffer_size);
    ESP_LOGI(TAG, "\"slope\": %f,", slope);
    ESP_LOGI(TAG, "\"intercept\": %f,", intercept);
    ESP_LOGI(TAG, "\"std_err\": %f,", std_err);
    ESP_LOGI(TAG, "\"EventBuffer\": [");
    for(int i = 0; i < buffer_size; i++){
        ESP_LOGI(TAG, "{");
        ESP_LOGI(TAG, "\"TX\": {\"MCU\": %f, \"UWB\": %f},", event_buffer[i].tx.mcu, event_buffer[i].tx.uwb);
        ESP_LOGI(TAG, "\"RX\": {\"MCU\": %f, \"UWB\": %f},", event_buffer[i].rx.mcu, event_buffer[i].rx.uwb);
        ESP_LOGI(TAG, "\"Adjusted RX\": {\"MCU\": %f, \"UWB\": %3.2f},", event_buffer[i].adjusted_rx.mcu, event_buffer[i].adjusted_rx.uwb);
        ESP_LOGI(TAG, "\"ToF\": %f,", event_buffer[i].tof_dtu);
        ESP_LOGI(TAG, "\"Is Init\": %d,", event_buffer[i].is_init);
        ESP_LOGI(TAG, "\"Is SS\": %d", event_buffer[i].is_SS);
        ESP_LOGI(TAG, "}%s", (i < buffer_size - 1) ? "," : "");
    }
    ESP_LOGI(TAG, "]");
    ESP_LOGI(TAG, "}");
    ESP_LOGI(TAG, "End of event buffer");
}

GT_event_t gt;

void addSyncData(time_stamps_t *twr){
    static event_pair_t event_buffer[LR_BUFFER_SIZE];
    static int buffer_index = -1;
    static int count = 0;

    ESP_LOGI(TAG, "Adding sync data, count = %d", count);

    // sanity check
    if (twr == nullptr) return;
    if(twr->is_SS) return;

    // add all events from the twr buffer to the event buffer
    count++;    
    Ts2events(twr, event_buffer, &buffer_index, LR_BUFFER_SIZE);
    
    // if we have enough data, we can compute the linear regression
    if (count < REQUESTED_RANGING_AMOUNT/2) return;
    // Linear regression variables
    double slopeA = 0.0;
    double interceptA = 0.0;
    double std_errA = 0.0;
    int buffer_size = MIN( count*3, LR_BUFFER_SIZE);
    // now we have to perform the linear regression to go from our mcu to their mcu
    // LinearRegression(event_buffer, buffer_size, &slope, &intercept, &std_err, filterFunctionDS);
    LinearRegression(event_buffer, buffer_size, &slopeA, &interceptA, &std_errA, invertedrestrictedFilterFunctionDS);
    double slopeB = 0.0;
    double interceptB = 0.0;
    double std_errB = 0.0;
    // now we have to perform the linear regression to go from our mcu to their mcu
    // LinearRegression(event_buffer, buffer_size, &slope, &intercept, &std_err, filterFunctionDS);
    LinearRegression(event_buffer, buffer_size, &slopeB, &interceptB, &std_errB, restrictedFilterFunctionDS);
    double slope = (slopeA+slopeB)/2;
    double intercept = (interceptA+interceptB)/2;
    double std_err = (std_errA+std_errB)/2;

    MixedEffectsResults res = fit_mixed_effects_EM(event_buffer, buffer_size);
    gt.MixedResults = res;
    gt.slope = slope;
    gt.intercept = intercept;
    gt.std_err = std_err;
    printMixedEffectsResults(res);
    setFakeEvent(true);
    ESP_LOGI(TAG, "Fake -----------------------------");
}

void PrintGTeventJson(){
    ESP_LOGI(TAG, "{");
    ESP_LOGI(TAG, "\"counter\": %" PRIu32",", gt.counter);
    ESP_LOGI(TAG, "\"mcu_ts\": %.17g,", gt.mcu_ts);
    ESP_LOGI(TAG, "\"slope\": %.17g,", gt.slope);
    ESP_LOGI(TAG, "\"intercept\": %.17g,", gt.intercept);
    ESP_LOGI(TAG, "\"std_err\": %.17g,", gt.std_err);
    ESP_LOGI(TAG, "\"MixedResults\": {");
    ESP_LOGI(TAG, "\"beta0\": %.17g,", gt.MixedResults.beta0);
    ESP_LOGI(TAG, "\"beta1\": %.17g,", gt.MixedResults.beta1);
    ESP_LOGI(TAG, "\"sigma2\": %.17g,", gt.MixedResults.sigma2);
    ESP_LOGI(TAG, "\"sigma_u2\": %.17g,", gt.MixedResults.sigma_u2);
    ESP_LOGI(TAG, "\"iterations\": %d,", gt.MixedResults.iterations);
    ESP_LOGI(TAG, "\"u_hat_0\": %.17g,", gt.MixedResults.u_hat[0]);
    ESP_LOGI(TAG, "\"u_hat_1\": %.17g,", gt.MixedResults.u_hat[1]);  
    ESP_LOGI(TAG, "},");
    ESP_LOGI(TAG, "\"temperature\": %.9g", gt.temperature);
    ESP_LOGI(TAG, "}");
}

void GT_Task(void*){
    temperature_sensor_handle_t temp_handle = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
    float tsens_out;
    for (;;) {
        // Wait for notification
        if(ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY)){
            ESP_LOGI(TAG, "GT Task");
            // Call get capture time
            uint64_t capture = get_capture_time();
            // Increment the counter in the GT event
            gt.counter++;
            // Set the mcu_ts from the capture
            // gt.mcu_ts = capture;
            gt.mcu_ts = mcu_to_us(capture);
            ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
            gt.temperature = tsens_out;
            //print the GT event
            PrintGTeventJson();
        }
    }
    vTaskDelete(NULL);
}

double mcu_to_us(uint64_t mcu_ticks){
    // 80Mhz = 12.5 ns per tick -> 1us = 80 ticks
    const double TICK_PERIOD_US = 1.0/80.0; //! <= 12.5 ns
    return ((double)mcu_ticks) * TICK_PERIOD_US;
}

double uwb_to_us(int64_t diff_in_ticks){
    // Each UWB tick is ~15.64 ps, which is 0.00001564 µs
    // so TICK_PERIOD_US = 1.564e-5
    const double TICK_PERIOD_US = (1.0 / 499.2e6 / 128.0) * 1e6; 
    return ((double)diff_in_ticks) * TICK_PERIOD_US;
}

static inline int64_t dw3000_40bit_diff(uint64_t t_new, uint64_t t_old){
    // We assume that only the lowest 40 bits of t_new and t_old are valid.
    // Mask off above 40 bits:
    const uint64_t MASK_40 = 0xFFFFFFFFFFULL;  // lower 40 bits
    uint64_t masked_new = t_new & MASK_40;
    uint64_t masked_old = t_old & MASK_40;
    
    // Compute raw 40-bit difference:
    uint64_t diff_40 = (masked_new - masked_old) & MASK_40;
    
    // If the 40-bit difference is above 2^39, we consider that a negative wrap.
    // (That is, the difference is actually diff_40 - 2^40.)
    // So we check bit 39 (counting from 0).
    const uint64_t HALF_RANGE = (1ULL << 39);

    if (diff_40 & HALF_RANGE) {
        // sign-extend in 64 bits
        diff_40 |= ~MASK_40;  // set the upper bits to 1
    }
    
    return (int64_t)diff_40; // now a signed 64-bit result
}

void printTimestamps(time_stamps_t twr){
    ESP_LOGI("TINY_SYNC", "SN: %u, other_addr: %X, SS: %d, Init? %d", twr.SN, twr.other_addr, twr.is_SS, twr.is_initiator);
    ESP_LOGI("TINY_SYNC", "Distance: %3.2f m", twr.distance);
    ESP_LOGI("TINY_SYNC", "UWB: Poll TX: %" PRIu64 ", Poll RX: %" PRIu64, twr.poll_tx.DW3000_time, twr.poll_rx.DW3000_time);
    ESP_LOGI("TINY_SYNC", "UWB:Resp TX: %" PRIu64 ", Resp RX: %" PRIu64, twr.resp_tx.DW3000_time, twr.resp_rx.DW3000_time);
    ESP_LOGI("TINY_SYNC", "UWB:Final TX: %" PRIu64 ", Final RX: %" PRIu64, twr.final_tx.DW3000_time, twr.final_rx.DW3000_time);
    ESP_LOGI("TINY_SYNC", "MCU: Poll TX: %" PRIu64 ", Poll RX: %" PRIu64, twr.poll_tx.MCU_time, twr.poll_rx.MCU_time);
    ESP_LOGI("TINY_SYNC", "MCU:Resp TX: %" PRIu64 ", Resp RX: %" PRIu64, twr.resp_tx.MCU_time, twr.resp_rx.MCU_time);
    ESP_LOGI("TINY_SYNC", "MCU:Final TX: %" PRIu64 ", Final RX: %" PRIu64, twr.final_tx.MCU_time, twr.final_rx.MCU_time);
    ESP_LOGI("TINY_SYNC", "-------------------------------");
}

void print_buffer(time_stamps_t *buffer, int buffer_size){
    for(int i = 0; i < buffer_size; i++){
        ESP_LOGI("TINY_SYNC", "SN: %u, other_addr: %u, SS: %d, Init? %d", buffer[i].SN, buffer[i].other_addr, buffer[i].is_SS, buffer[i].is_initiator);
    }
    ESP_LOGI("TINY_SYNC", "-------------------------------");
}

int find_matching(time_stamps_t reference, time_stamps_t *buffer, int buffer_size){
    uint64_t diff = 0;
    for (int i = 0; i < buffer_size; i++) {
        if (buffer[i].SN == reference.SN &&
            buffer[i].is_initiator != reference.is_initiator //&&
            //buffer[i].final_tx.DW3000_time == reference.final_tx.DW3000_time) {
            ){
            // Check if the timestamps are complementary
            if(buffer[i].final_tx.DW3000_time >= reference.final_tx.DW3000_time){
                diff = buffer[i].final_tx.DW3000_time - reference.final_tx.DW3000_time;
            } else {
                diff = reference.final_tx.DW3000_time - buffer[i].final_tx.DW3000_time;
            }
            ESP_LOGI("FIND_MATCH", "Diff: %llu", diff);
            if (diff>30){
                continue;
            }
            // printTimestamps(buffer[i]);
            // printTimestamps(reference);
            return i;  // Found a complementary timestamp
        }
    }
    return -1;  // No complementary timestamp found
}

void periodic_request_Task(void *pvParameters){
    const int threshold = REQUESTED_RANGING_AMOUNT*1000000; // in us
    uint64_t sync_attempt_ts = esp_timer_get_time()-threshold/10;
    int request = REQUESTED_RANGING_AMOUNT;
    ESP_LOGI("PERIODIC_REQ", "Starting periodic request task");
    for(;;){
        int nmsgsinQ = uxQueueMessagesWaiting(ranging_request_Q);
        if(nmsgsinQ > 0) {
            ESP_LOGI("PERIODIC_REQ", "Ranging request already in queue %d", nmsgsinQ);
            sync_attempt_ts = esp_timer_get_time();
            vTaskDelay(pdMS_TO_TICKS(threshold/1000)); // wait until the next attempt window
            continue;
        }

        uint64_t curr_ts = esp_timer_get_time();
        int diff_attempt = (curr_ts - sync_attempt_ts);
        ESP_LOGI("PERIODIC_REQ", "diff: %d", diff_attempt);
        if(diff_attempt > threshold){
            // request a new set of ranging results
            xQueueSendToBack(ranging_request_Q, &request, 0);
            sync_attempt_ts = curr_ts;
        } else {
            vTaskDelay(pdMS_TO_TICKS(diff_attempt / 1000)); // wait until the next attempt window
        }
    }
    vTaskDelete(NULL);
}

void coordinator_decision(bool our_side, time_stamps_t current, time_stamps_t* storage_buffer, time_stamps_t* compare_buffer, int *storage_index, int buffer_size){
    // check if we have a complentary timestamp
    int index = find_matching(current, compare_buffer, buffer_size);
    if(index>=0){
        time_stamps_t other = our_side? compare_buffer[index] : current;
        time_stamps_t us = our_side? current : compare_buffer[index];
        ESP_LOGI("COORD_SYNC", "Found matching timestamp");
        // TODO: send both to processing queue
        // mix
        time_stamps_t mix = us;
        // fill the mix with info of both 
        if(mix.is_initiator){
            mix.poll_tx = us.poll_tx;
            mix.poll_rx = other.poll_rx;
            mix.resp_tx = other.resp_tx;
            mix.resp_rx = us.resp_rx;
            mix.final_tx = us.final_tx;
            mix.final_rx = other.final_rx;
        } else {
            mix.poll_tx = other.poll_tx;
            mix.poll_rx = us.poll_rx;
            mix.resp_tx = us.resp_tx;
            mix.resp_rx = other.resp_rx;
            mix.final_tx = other.final_tx;
            mix.final_rx = us.final_rx;
        }
        // printTimestamps(mix);
        addSyncData(&mix);

    }else {
        ESP_LOGI("COORD_SYNC", "No matching timestamp found");
        // printTimestamps(current);
        // print_buffer(storage_buffer, buffer_size);

        storage_buffer[*storage_index] = current;
        *storage_index = (*storage_index + 1) % buffer_size;
    }
}

void sync_coordinator_Task(void *pvParameters){
    time_stamps_t curr_twr;

    ESP_LOGI("COORD_SYNC", "Starting sync coordinator task");
    QueueSetMemberHandle_t QueueEvent;
    QueueSetHandle_t xQueueSet = xQueueCreateSet( QUEUE_SET_LENGTH );
    xQueueAddToSet(ranging_result_Q, xQueueSet);
    xQueueAddToSet(receive_lora_Q, xQueueSet);

    static time_stamps_t local_buffer[TS_RING_BUFFER_SIZE];
    int local_buffer_index = 0;
    static time_stamps_t remote_buffer[TS_RING_BUFFER_SIZE];
    int remote_buffer_index = 0;
    int sentcounter = 0;
    for(;;){
        // wait for an event
        // ESP_LOGI("COORD_SYNC", "Waiting for event");
        QueueEvent = xQueueSelectFromSet(xQueueSet, portMAX_DELAY);

        if(QueueEvent == ranging_result_Q){
            if (xQueueReceive(ranging_result_Q, &curr_twr, 0)) {

                // ESP_LOGI("COORD_SYNC", "Received from ranging_result_Q");
                // printTimestamps(curr_twr);
                // ? Do we make decisions on reception of our own timestamps?
                coordinator_decision(true, curr_twr, local_buffer, remote_buffer, &local_buffer_index, TS_RING_BUFFER_SIZE);
                // we have to send the timestamp to the other side
                int ret = xQueueSendToBack(emit_lora_Q, &curr_twr, 0); // encoding done by sending task
                if(ret != pdTRUE) {
                    ESP_LOGE("COORD_SYNC", "Error sending to emit_lora_Q");
                }else {
                    sentcounter++;
                    // ESP_LOGI("COORD_SYNC", "Sent %d timestamps", sentcounter);
                }
            } else {
                ESP_LOGE("COORD_SYNC", "Error receiving from ranging_result_Q");
            }
        }
        if(QueueEvent == receive_lora_Q){
            if (xQueueReceive(receive_lora_Q, &curr_twr, 0)) {

                // ESP_LOGI("COORD_SYNC", ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Received from receive_lora_Q");
                // printTimestamps(curr_twr);
                coordinator_decision(false, curr_twr, remote_buffer, local_buffer, &remote_buffer_index, TS_RING_BUFFER_SIZE);
            } else {
                ESP_LOGE("COORD_SYNC", "Error receiving from receive_lora_Q");
            }
        }
    }
    vTaskDelete(NULL);
}

TaskHandle_t create_sync_task(QueueHandle_t Q_ranging_result, QueueHandle_t Q_ranging_request, QueueHandle_t Q_emit_lora,  QueueHandle_t Q_receive_lora){
    TaskHandle_t xTask;
    ranging_request_Q = Q_ranging_request;
    ranging_result_Q = Q_ranging_result;
    emit_lora_Q = Q_emit_lora;
    receive_lora_Q = Q_receive_lora;
    // xTaskCreate(periodic_request_Task, "period ranging req", 4096, NULL, 3, NULL);
    xTaskCreate(sync_coordinator_Task, "tiny coord Task", 8192, NULL, 6, NULL);
    xTaskCreate(GT_Task, "GT Task", 4096, NULL, 5, &xTask);
    return xTask;
}