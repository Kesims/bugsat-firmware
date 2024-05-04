#ifndef STM32_DEVICE_CONFIG_H
#define STM32_DEVICE_CONFIG_H

#include <stdbool.h>
#include "main.h"

#define CRC32_POLYNOMIAL 0xEDB88320
#define DEVICE_CONFIG_ADDRESS 0x0808F800


typedef struct DeviceConfig {
    uint32_t lora_frequency;
    uint32_t lora_bandwidth;
    uint16_t lora_sync_word;
    uint8_t lora_spreading_factor;
    uint8_t lora_tx_power;

    uint32_t crc32;
} DeviceConfig;

void write_config_to_flash();
void load_device_config();

uint32_t calculate_crc32(const uint8_t *data, uint32_t length);
uint8_t resolve_lora_tx_power(uint8_t tx_power);
uint8_t resolve_lora_spreading_factor(uint8_t spreading_factor);
uint32_t resolve_lora_bandwidth(uint32_t bandwidth);

bool set_lora_frequency(uint32_t frequency);
bool set_lora_tx_power(uint8_t tx_power);
bool set_lora_bandwidth(uint32_t bandwidth);
bool set_lora_sync_word(uint16_t sync_word);
bool set_lora_spreading_factor(uint8_t spreading_factor);


#endif //STM32_DEVICE_CONFIG_H
