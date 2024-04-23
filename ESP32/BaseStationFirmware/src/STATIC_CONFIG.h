#ifndef BASESTATIONFIRMWARE_STATIC_CONFIG_H
#define BASESTATIONFIRMWARE_STATIC_CONFIG_H

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 4
#define LORA_DIO0 33


#define LORA_SPREADING_FACTOR 10  //Spreading factor 1 - 12, higher value => lower speeds and higher link budget
// recommended 9-10
#define LORA_FREQUENCY 433E6 // Frequency in Hz
#define LORA_SIGNAL_BANDWIDTH 125E3 // Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
// 41.7E3, 62.5E3, 125E3, 250E3, and 500E3
#define LORA_TX_POWER 20  // Power in dBm, supported values are 6-20
#define LORA_SYNC_WORD 0x79 // Sync word



#endif //BASESTATIONFIRMWARE_STATIC_CONFIG_H
