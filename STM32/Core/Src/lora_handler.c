#include <math.h>
#include <stdio.h>
#include <string.h>
#include "lora_handler.h"
#include "SX1278.h"
#include "cmsis_os.h"
#include "battery_voltage.h"
#include "sensors_handler.h"
#include "gps_driver.h"
#include "device_config.h"

SX1278_hw_t SX1278_hw;
SX1278_t SX1278;
extern SPI_HandleTypeDef hspi1;
extern osSemaphoreId GPS_Buffer_SemaphoreHandle;
extern osSemaphoreId Sensor_Buffer_SemaphoreHandle;
extern DeviceConfig device_config;

volatile bool lora_send_gps = false;
volatile bool lora_send_sensor = false;
volatile bool lora_send_battery = false;
volatile bool lora_send_status = false;

extern BPM390BufferData bmp390_data;
extern LIS3DHBufferData lis3dh_highg;
extern GPSData gps_buffer;

static uint8_t packet_id = 0;

static uint8_t get_next_packet_id() {
    return packet_id++;
}

void lora_handler_init() {
    // Prepare the LoRa SX1278 module
    //initialize LoRa module
    SX1278_hw.dio0.port = LORA_DIO0_GPIO_Port;
    SX1278_hw.dio0.pin = LORA_DIO0_Pin;
    SX1278_hw.nss.port = LORA_NSS_GPIO_Port;
    SX1278_hw.nss.pin = LORA_NSS_Pin;
    SX1278_hw.reset.port = LORA_RST_GPIO_Port;
    SX1278_hw.reset.pin = LORA_RST_Pin;
    SX1278_hw.spi = &hspi1;

    SX1278.hw = &SX1278_hw;

    SX1278_init_with_sync_word(&SX1278,
                               device_config.lora_frequency,
                               resolve_lora_tx_power(device_config.lora_tx_power),
                               resolve_lora_spreading_factor(device_config.lora_spreading_factor),
                               resolve_lora_bandwidth(device_config.lora_bandwidth),
                               SX1278_LORA_CR_4_5,
                               SX1278_LORA_CRC_EN,
                               10,
                               device_config.lora_sync_word);

    SX1278_LoRaEntryTx(&SX1278, 16, 2000);
}

uint8_t determine_packet_length(enum PacketType packetType) {
    switch (packetType) {
        case BATTERY_DATA:
            return sizeof(LoraBatteryData);
        case GPS_DATA:
            return sizeof(LoraGPSData);
        case SENSOR_DATA:
            return sizeof(LoraSensorData);
        case STATUS_DATA:
            return sizeof(LoraStatusData);
        default:
            return 0;
    }
}

void send_packet(LoraPacket packet) {
    uint8_t message_length = determine_packet_length(packet.packetType) + 2; // 2 is packet header
    // Prepare the packet
    SX1278_LoRaEntryTx(&SX1278, message_length, 250);
    // Send packet
    uint8_t packet_buffer[message_length];
    packet_buffer[0] = packet.packetId;
    packet_buffer[1] = packet.packetType;
    memcpy(&packet_buffer[2], packet.data, message_length - 2);
    SX1278_LoRaTxPacket(&SX1278, packet_buffer,
                        message_length, 250);
    osDelay(25);
}

void send_gps_data() {
    LoraGPSData gps_data = {0};

//    if(xSemaphoreTake(GPS_Buffer_SemaphoreHandle, 120) == pdTRUE) {
        gps_data.seconds_since_midnight = gps_buffer.seconds_since_midnight;
        gps_data.latitude = gps_buffer.latitude;
        gps_data.longitude = gps_buffer.longitude;
        gps_data.altitude = gps_buffer.altitude;
        gps_data.hdop = gps_buffer.hdop;
//        xSemaphoreGive(GPS_Buffer_SemaphoreHandle);

        LoraPacket packet;
        packet.packetId = get_next_packet_id();
        packet.packetType = GPS_DATA;
        packet.data = (uint8_t*) &gps_data;

        send_packet(packet);
//    }
}


void send_sensor_data() {
    LoraSensorData sensor_data = {0};

    uint16_t converted_temperature = (uint16_t) roundf(bmp390_data.temperature * 100);
//    if(xSemaphoreTake(Sensor_Buffer_SemaphoreHandle, 120) == pdTRUE) {
        sensor_data.temperature = converted_temperature;
        sensor_data.pressure = bmp390_data.pressure;
        sensor_data.accelerationX = lis3dh_highg.accelerationX;
        sensor_data.accelerationY = lis3dh_highg.accelerationY;
        sensor_data.accelerationZ = lis3dh_highg.accelerationZ;
//        xSemaphoreGive(Sensor_Buffer_SemaphoreHandle);

        LoraPacket packet;
        packet.packetId = get_next_packet_id();
        packet.packetType = SENSOR_DATA;
        packet.data = (uint8_t*) &sensor_data;

        send_packet(packet);
//    }
}


void send_battery_data() {
    LoraBatteryData battery_data = {0};

    battery_data.voltage = get_battery_voltage();

    LoraPacket packet;
    packet.packetId = get_next_packet_id();
    packet.packetType = BATTERY_DATA;
    packet.data = (uint8_t*) &battery_data;

    send_packet(packet);
}

void send_status_data() {
    LoraStatusData status_data = {0};


    LoraPacket packet;
    packet.packetId = get_next_packet_id();
    packet.packetType = STATUS_DATA;
    packet.data = (uint8_t*) &status_data;

    send_packet(packet);
}

_Noreturn void lora_task_work() {
    for (;;) {
        if (lora_send_gps) {
            send_gps_data();
            lora_send_gps = false;
        }
        if (lora_send_sensor) {
            send_sensor_data();
            lora_send_sensor = false;
        }
        if (lora_send_battery) {
            send_battery_data();
            lora_send_battery = false;
        }
        if (lora_send_status) {
            send_status_data();
            lora_send_status = false;
        }
        osDelay(50);
    }
}






