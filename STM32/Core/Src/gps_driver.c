#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "gps_driver.h"
#include "cmsis_os.h"

#define MAX_NMEA_SENTENCE_LENGTH 100

extern osSemaphoreId GPS_Task_SemaphoreHandle;
char gps_rx_buffer[MAX_NMEA_SENTENCE_LENGTH];
int sentence_index = 0;
bool sentence_started = false;


// Typical NMEA format is:
// $GNGGA,072446.00,3130.5226316,N,12024.0937010,E,4,27,0.5,31.924,M,0.000,M,2.0,*44
// Each message starting with $ and ending with \n


void parse_GNGGA(char* nmea_sentence, GNGGA_Info* info) {
    char* token;
    int field_count = 0;

    token = strtok(nmea_sentence, ",");

    while (token != NULL) {
        switch (field_count) {
            case 1: // Time
                strncpy(info->time, token, sizeof(info->time) - 1);
                info->time[sizeof(info->time) - 1] = '\0';
                break;
            case 2: // Latitude
                info->latitude = atof(token);
                break;
            case 4: // Longitude
                info->longitude = atof(token);
                break;
            case 7: // Number of satellites
                info->satellites = atoi(token);
                break;
            case 9: // Altitude
                info->altitude = atof(token);
                break;
            case 8: // HDOP
                info->hdop = atof(token);
                break;
            default:
                break;
        }

        token = strtok(NULL, ",");
        field_count++;
    }
}

void gps_uart_callback(char rx_char) {
    // Start of sentence
    if (rx_char == '$') {
        sentence_started = true;
        sentence_index = 0;
    }

    // If sentence has started, add character to buffer
    if (sentence_started) {
        gps_rx_buffer[sentence_index] = rx_char;
        sentence_index++;

        // End of sentence
        if (rx_char == '\n') {
            sentence_started = false;
            gps_rx_buffer[sentence_index] = '\0'; // Null-terminate the sentence

            // Signal the FreeRTOS task
            xSemaphoreGiveFromISR(GPS_Task_SemaphoreHandle, NULL);

            // Reset index for next sentence
            sentence_index = 0;
        }

        // Check if buffer is full
        if (sentence_index >= MAX_NMEA_SENTENCE_LENGTH) {
            // Buffer is full, reset everything
            sentence_started = false;
            sentence_index = 0;
        }
    }
}

_Noreturn void gps_parsing_task_work() {
    GNGGA_Info info = {0};

    for(;;) {
        // Wait for a signal from the callback function
        if (xSemaphoreTake(GPS_Task_SemaphoreHandle, portMAX_DELAY) == pdTRUE) {
            // Parse the NMEA sentence
            parse_GNGGA(gps_rx_buffer, &info);

            // Now do stuff with the parsed data
        }
    }
}