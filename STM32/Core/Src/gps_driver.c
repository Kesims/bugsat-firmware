#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "gps_driver.h"
#include "cmsis_os.h"
#include "debug_printf.h"
#include "bug_deployment.h"
#include "uart_nrf52.h"

#define MAX_NMEA_SENTENCE_LENGTH 100

extern osSemaphoreId GPS_Task_SemaphoreHandle;
extern osSemaphoreId GPS_Buffer_SemaphoreHandle;
char gps_rx_buffer[MAX_NMEA_SENTENCE_LENGTH];
int sentence_index = 0;
bool sentence_started = false;


// Typical NMEA format is:
// $GNGGA,072446.00,3130.5226316,N,12024.0937010,E,4,27,0.5,31.924,M,0.000,M,2.0,*44
// Each message starting with $ and ending with \n

GPSData gps_buffer = {0};

void parse_GNGGA(char* nmea_sentence, GNGGA_Info* info) {

    // Check if the sentence is a GNGGA sentence
    if (strncmp(nmea_sentence, "$GNGGA", 6) != 0) {
        // Not a GNGGA sentence, return immediately
        return;
    }


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

// Convert time in HHMMSS format to seconds since midnight
static uint32_t time_to_seconds(const char* time) {
    uint16_t hours = (time[0] - '0') * 10 + (time[1] - '0');
    uint16_t minutes = (time[2] - '0') * 10 + (time[3] - '0');
    uint16_t seconds = (time[4] - '0') * 10 + (time[5] - '0');

    return hours * 3600 + minutes * 60 + seconds;
}

_Noreturn void gps_parsing_task_work() {
    GNGGA_Info info = {0};

    for(;;) {
        // Wait for a signal from the callback function
        if (xSemaphoreTake(GPS_Task_SemaphoreHandle, portMAX_DELAY) == pdTRUE) {
            // Parse the NMEA sentence
            parse_GNGGA(gps_rx_buffer, &info);

            // Now do stuff with the parsed data
            // For now, just print it
//            debugPrintf("Time: %s\n", info.time);
//            osDelay(2);
//            debugPrintf("Latitude: %f\n", info.latitude);
//            osDelay(2);
//            debugPrintf("Longitude: %f\n", info.longitude);
//            osDelay(2);
//            debugPrintf("Satellites: %d\n", info.satellites);
//            osDelay(2);
//            debugPrintf("Altitude: %f\n", info.altitude);
//            osDelay(2);
//            debugPrintf("HDOP: %f\n", info.hdop);
//            osDelay(2);

            // Convert time to seconds since midnight
//            if( xSemaphoreTake(GPS_Buffer_SemaphoreHandle, portMAX_DELAY) == pdTRUE) {


                // A thing that handles bug deployment and definitely belongs into this module
                add_altitude_to_buffer((float) info.altitude);
                check_start_deployment_process();

            // Check if altitude has changed from 0 to a value or from a value to 0
                if ((gps_buffer.altitude == 0 && info.altitude != 0) ||
                    (gps_buffer.altitude != 0 && info.altitude == 0)) {
                    uart_send_bugsat_status();
                }


                gps_buffer.seconds_since_midnight = time_to_seconds(info.time);
                gps_buffer.latitude = (float) info.latitude;
                gps_buffer.longitude = (float) info.longitude;
                gps_buffer.altitude = (uint16_t) info.altitude;
                gps_buffer.hdop = (float) info.hdop;
                gps_buffer.satellites = (uint8_t) info.satellites;
//                xSemaphoreGive(GPS_Buffer_SemaphoreHandle);
//            }
        }
    }
}

void disable_unused_nmea_sentences(UART_HandleTypeDef* huart) {
    // List of unused NMEA sentences
    const char* unused_sentences[] = {
            "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n", // Disable GLL
            "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n", // Disable GSA
            "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n", // Disable GSV
            "$PUBX,40,RMC,0,0,0,0,0,0*47\r\n", // Disable RMC
            "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n", // Disable VTG
    };

    for (int i = 0; i < sizeof(unused_sentences) / sizeof(unused_sentences[0]); i++) {
        // Send the command to the GPS module
        HAL_UART_Transmit(huart, (uint8_t*)unused_sentences[i], strlen(unused_sentences[i]), 200);
        osDelay(50);
    }
}