#include "lora_handler.h"

#include <Arduino.h>
#include <WString.h>
#include "lora_handler.h"
#include "LoRa.h"
#include "debug.h"
#include "STATIC_CONFIG.h"
#include "wifi_handler.h"
#include "LinkedList.h"

uint64_t last_lora_receive = 0;
int last_rssi = -1;
float last_snr = -1;

LoraPacket receive_packet;
uint8_t packet_data_buffer[255];

LinkedList<uint8_t *> unprocessed_packets = LinkedList<uint8_t *>();


void lora_init() {
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    delay(100);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        debugPrintln(F("Lora failed to start!!"));
        while (1);
    }

    // Uncomment the next line to disable the default AGC and set LNA gain, values between 1 - 6 are supported
    // LoRa.setGain(6);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    debugPrintln("Spreading factor set to: " + String(LORA_SPREADING_FACTOR));
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.enableCrc(); // Filter corrupt packets
    //LoRa.channelActivityDetection(); // Enable detection for other traffic (detects preambles)
    LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH); // Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
    // 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
    // put the radio into receive mode
    delay(10);
    LoRa.receive();
    check_lora_receiver();
}


void check_lora_receiver() {
    on_lora_receive(LoRa.parsePacket());
}

uint8_t get_packet_length_by_type(enum PacketType packetType) {
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

void add_packet_to_unprocessed(LoraPacket packet) {
    auto * packet_data = (uint8_t*) malloc(get_packet_length_by_type((PacketType) packet.packetType));
    memcpy(packet_data, packet.data, get_packet_length_by_type((PacketType) packet.packetType));
    auto * unprocessed_packet = (LoraPacket*) malloc(sizeof(LoraPacket));
    unprocessed_packet->packetId = packet.packetId;
    unprocessed_packet->packetType = packet.packetType;
    unprocessed_packet->data = packet_data;
    unprocessed_packets.add((uint8_t *) unprocessed_packet);
}

void on_lora_receive(int packetSize) {
    if (packetSize == 0) return; // If there is no packet, do nothing

    if(!LoRa.available()) return;

//    debugPrintln("Received packet!");

//    while (LoRa.available()) {
//        debugPrintln("Received char: " + String(LoRa.read()));
//    }
//    return;


    // Clear the receive buffer
    receive_packet = {0};
    memset(packet_data_buffer, 0, 255);

    // Read the packet ID
    receive_packet.packetId = LoRa.read();
    // Read the packet type
    receive_packet.packetType = LoRa.read();

    // Check that the packet type is valid
    if (receive_packet.packetType >= PACKET_TYPE_MAX_VAL) {
        debugPrintln("Invalid packet type: " + String(receive_packet.packetType));
        debugPrintln("Packet type: " + String(receive_packet.packetType) + " Packet ID: " + String(receive_packet.packetId));
        // Clean the reception buffer
        while (LoRa.available()) {
            LoRa.read();
        }
        return;
    }

    // read the data
    uint8_t packet_length = get_packet_length_by_type((PacketType) receive_packet.packetType);
    for (int i = 0; i < packet_length; i++) {
        packet_data_buffer[i] = LoRa.read();
//        debugPrintln("read: " + String(packet_data_buffer[i]));
    }

    receive_packet.data = packet_data_buffer;

    if (LoRa.available()) {
        debugPrintln("Packet too long, discarding!");
        debugPrintln("Packet type: " + String(receive_packet.packetType) + " Packet ID: " + String(receive_packet.packetId));
        // Clean the reception buffer
        while (LoRa.available()) {
            LoRa.read();
        }
        return;
    }

    add_packet_to_unprocessed(receive_packet);

    last_lora_receive = millis();
    last_rssi = LoRa.packetRssi();
    last_snr = LoRa.packetSnr();

    debugPrintln("RSSI: " + String(LoRa.packetRssi()) + " SNR: " + String(LoRa.packetSnr()));
}

uint16_t time_since_last_lora_receive() {
    return millis() - last_lora_receive;
}

void process_received_packet(LoraPacket packet) {
    switch (packet.packetType) {
        case BATTERY_DATA: {
//            debugPrintln("Received battery data!");
            LoraBatteryData* battery_data = (LoraBatteryData*) packet.data;
            upload_battery_data(packet.packetId, *battery_data);
            break;
        }
        case GPS_DATA: {
//            debugPrintln("Received GPS data!");
            LoraGPSData* gps_data = (LoraGPSData*) packet.data;
            upload_gps_data(packet.packetId, *gps_data);
            break;
        }
        case SENSOR_DATA: {
//            debugPrintln("Received sensor data!");
            LoraSensorData* sensor_data = (LoraSensorData*) packet.data;
            upload_sensor_data(packet.packetId, *sensor_data);
            break;
        }
        case STATUS_DATA: {
//            debugPrintln("Received status data!");
            LoraStatusData* status_data = (LoraStatusData*) packet.data;
            upload_status_data(packet.packetId, *status_data);
            break;
        }
        case BUGPACK_L_DATA: {
            LoraBugpackData* bugpack_data = (LoraBugpackData*) packet.data;
            upload_bugpack_data(packet.packetId, *bugpack_data);
            break;
        }
        default:
            break;
    }
}

void handle_unprocessed_packets() {
    while(unprocessed_packets.size() > 0) {
        auto * unprocessed_packet = (LoraPacket*) unprocessed_packets.shift();
        LoraPacket packet = *unprocessed_packet;
        process_received_packet(packet);
        delete[](unprocessed_packet->data);
        delete[](unprocessed_packet);
    }
}



