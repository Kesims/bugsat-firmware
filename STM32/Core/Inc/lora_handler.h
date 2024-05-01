#ifndef STM32_LORA_HANDLER_H
#define STM32_LORA_HANDLER_H

#include "main.h"

enum PacketType {
    BATTERY_DATA = 0,
    GPS_DATA = 1,
    SENSOR_DATA = 2,
    STATUS_DATA = 3
};

typedef struct LoraGPSData { // send every 5 seconds
    uint32_t seconds_since_midnight;
    float latitude;
    float longitude;
    uint16_t altitude;
    float hdop;
} LoraGPSData;

typedef struct LoraSensorData { // send every second
    uint16_t temperature; // 365.88 deg C -> 36588
    float pressure;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
} LoraSensorData;

typedef struct LoraBatteryData { // send every minute
    uint16_t voltage; // mV
} LoraBatteryData;

typedef struct LoraStatusData { // send every minute
    uint8_t status; // first bit: GPS ready, second bit: trigger done
} LoraStatusData;

typedef struct LoraPacket {
    uint8_t packetId;
    uint8_t packetType; // Packet type also determines the data length
    uint8_t* data;
} LoraPacket;

void send_gps_data();
void send_sensor_data();
void send_battery_data();
void send_status_data();

_Noreturn void lora_task_work();
void lora_handler_init();


#endif //STM32_LORA_HANDLER_H
