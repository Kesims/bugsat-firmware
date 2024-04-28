#include "lora_handler.h"

#include <Arduino.h>
#include <WString.h>
#include "lora_handler.h"
#include "LoRa.h"
#include "debug.h"
#include "STATIC_CONFIG.h"

uint64_t last_lora_receive = 0;
int last_rssi = -1;
float last_snr = -1;

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

void on_lora_receive(int packetSize) {
    if (packetSize == 0) return; // If there is no packet, do nothing

    if(!LoRa.available()) return;

    String incoming = "";
    while (LoRa.available()) {
        incoming += (char)LoRa.read();
    }

    last_lora_receive = millis();
    last_rssi = LoRa.packetRssi();
    last_snr = LoRa.packetSnr();

    debugPrintln("Received packet: " + incoming);
    debugPrintln("RSSI: " + String(LoRa.packetRssi()) + " SNR: " + String(LoRa.packetSnr()));
}

uint16_t time_since_last_lora_receive() {
    return millis() - last_lora_receive;
}



