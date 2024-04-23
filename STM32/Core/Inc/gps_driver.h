#ifndef STM32_GPS_DRIVER_H
#define STM32_GPS_DRIVER_H

typedef struct {
    char time[11]; // HHMMSS.SSS
    double latitude;
    double longitude;
    int satellites;
    double altitude;
    double hdop; // Horizontal Dilution of Precision
} GNGGA_Info;

#endif //STM32_GPS_DRIVER_H
