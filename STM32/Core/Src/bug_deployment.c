#include "bug_deployment.h"
#include "indicator_utils.h"
#include "stm32l4xx_hal.h"
#include "uart_nrf52.h"
#include <stdbool.h>

#define BUFFER_SIZE 60
#define ABOVE_AVERAGE_THRESHOLD 250
#define ABOVE_AVERAGE_COUNT_THRESHOLD 10

bool rocket_launch_detected = false;
bool bugs_deployed = false;

extern TIM_HandleTypeDef htim15;

typedef struct CircularBuffer {
    float buffer[BUFFER_SIZE];
    int start;
    int end;
} CircularBuffer;

CircularBuffer gps_alt_buffer = {0};

void add_altitude_to_buffer(float value) {
    gps_alt_buffer.buffer[gps_alt_buffer.end] = value;
    gps_alt_buffer.end = (gps_alt_buffer.end + 1) % BUFFER_SIZE;
    if (gps_alt_buffer.end == gps_alt_buffer.start) {
        gps_alt_buffer.start = (gps_alt_buffer.start + 1) % BUFFER_SIZE; // full, overwrite
    }
}

float calculate_altitude_average() {
    float sum = 0.0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += gps_alt_buffer.buffer[i];
    }
    return sum / BUFFER_SIZE;
}

bool is_gps_getting_data() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (gps_alt_buffer.buffer[i] != 0.0) {
            return true;
        }
    }
    return false;
}

bool detect_rocket_launch() {
    if (!is_gps_getting_data()) {
        return false;
    }

    float average = calculate_altitude_average();
    int above_average_count = 0;

    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (gps_alt_buffer.buffer[i] > average + ABOVE_AVERAGE_THRESHOLD) {
            above_average_count++;
            if (above_average_count >= ABOVE_AVERAGE_COUNT_THRESHOLD) {
                return true;
            }
        } else {
            above_average_count = 0;
        }
    }

    return false;
}

void check_start_deployment_process() {
    if (detect_rocket_launch()) {
        HAL_TIM_Base_Start_IT(&htim15);
        rocket_launch_detected = true;
        indicate_info();
        uart_send_bugsat_status();
    }
}