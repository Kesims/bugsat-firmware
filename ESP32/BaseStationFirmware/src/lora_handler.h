#ifndef BASESTATIONFIRMWARE_LORA_HANDLER_H
#define BASESTATIONFIRMWARE_LORA_HANDLER_H

#include <Arduino.h>

enum PacketType {
    BATTERY_DATA = 0,
    GPS_DATA = 1,
    SENSOR_DATA = 2,
    STATUS_DATA = 3,
    PACKET_TYPE_MAX_VAL = 4
};

typedef struct LoraGPSData { // send every 5 seconds
    uint32_t seconds_since_midnight;
    float latitude;
    float longitude;
    uint16_t altitude;
    float hdop;
} LoraGPSData;

typedef struct LoraSensorData { // send every second
    uint16_t temperature;
    float pressure;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
} LoraSensorData;

typedef struct LoraBatteryData { // send every minute
    uint16_t voltage; // mV
} LoraBatteryData;

typedef struct LoraStatusData { // send every minute
    uint8_t status; // first bit: GPS ready,
                    // second bit: detected landing,
                    // third bit: trigger fired
} LoraStatusData;

typedef struct LoraPacket {
    uint8_t packetId;
    uint8_t packetType; // Packet type also determines the data length
    uint8_t* data;
} LoraPacket;


void check_lora_receiver();
void lora_init();
void on_lora_receive(int packetSize);
void process_received_packet(LoraPacket packet);
void handle_unprocessed_packets();

#endif //BASESTATIONFIRMWARE_LORA_HANDLER_H
