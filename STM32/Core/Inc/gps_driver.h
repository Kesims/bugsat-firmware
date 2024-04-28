#ifndef STM32_GPS_DRIVER_H
#define STM32_GPS_DRIVER_H

#include "main.h"


typedef struct {
    char time[11]; // HHMMSS.SSS
    double latitude;
    double longitude;
    int satellites;
    double altitude;
    double hdop; // Horizontal Dilution of Precision
} GNGGA_Info;

typedef struct GPSData {
    uint16_t seconds_since_midnight;
    float latitude;
    float longitude;
    uint16_t altitude;
    float hdop;
} GPSData;


void gps_uart_callback(char rx_char);
_Noreturn void gps_parsing_task_work();
void disable_unused_nmea_sentences(UART_HandleTypeDef* huart);

#endif //STM32_GPS_DRIVER_H
