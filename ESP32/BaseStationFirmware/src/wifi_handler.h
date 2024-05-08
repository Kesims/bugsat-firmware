#ifndef BASESTATIONFIRMWARE_WIFI_HANDLER_H
#define BASESTATIONFIRMWARE_WIFI_HANDLER_H

#include "Arduino.h"
#include "lora_handler.h"

void keep_wifi_alive();
void upload_status_data(uint8_t packet_id, LoraStatusData status_data);
void upload_gps_data(uint8_t packet_id, LoraGPSData gps_data);
void upload_sensor_data(uint8_t packet_id, LoraSensorData sensor_data);
void upload_battery_data(uint8_t packet_id, LoraBatteryData battery_data);
void upload_bugpack_data(uint8_t packet_id, LoraBugpackData bugpack_data);

#endif //BASESTATIONFIRMWARE_WIFI_HANDLER_H
