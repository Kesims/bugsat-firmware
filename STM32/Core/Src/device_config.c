#include <string.h>
#include "device_config.h"
#include "SX1278.h"
#include "indicator_utils.h"

DeviceConfig device_config = {
        .lora_frequency = 433000000,
        .lora_tx_power = 14,
        .lora_bandwidth = 250000,
        .lora_sync_word = 0x79,
        .lora_spreading_factor = 9,
        .crc32 = 0
};

uint8_t resolve_lora_tx_power(uint8_t tx_power) {
    if(tx_power <= 11) return SX1278_POWER_11DBM;
    if(tx_power <= 14) return SX1278_POWER_14DBM;
    if(tx_power <= 17) return SX1278_POWER_17DBM;
    if(tx_power <= 20) return SX1278_POWER_20DBM;
    return SX1278_POWER_14DBM;
}

uint8_t resolve_lora_spreading_factor(uint8_t spreading_factor) {
    switch (spreading_factor) {
        case 6:
            return SX1278_LORA_SF_6;
        case 7:
            return SX1278_LORA_SF_7;
        case 8:
            return SX1278_LORA_SF_8;
        case 9:
            return SX1278_LORA_SF_9;
        case 10:
            return SX1278_LORA_SF_10;
        case 11:
            return SX1278_LORA_SF_11;
        case 12:
            return SX1278_LORA_SF_12;
        default:
            return SX1278_LORA_SF_9;
    }
}

uint32_t resolve_lora_bandwidth(uint32_t bandwidth) {
    if(bandwidth <= 7800) return SX1278_LORA_BW_7_8KHZ;
    if(bandwidth <= 10400) return SX1278_LORA_BW_10_4KHZ;
    if(bandwidth <= 15600) return SX1278_LORA_BW_15_6KHZ;
    if(bandwidth <= 20800) return SX1278_LORA_BW_20_8KHZ;
    if(bandwidth <= 31200) return SX1278_LORA_BW_31_2KHZ;
    if(bandwidth <= 41700) return SX1278_LORA_BW_41_7KHZ;
    if(bandwidth <= 62500) return SX1278_LORA_BW_62_5KHZ;
    if(bandwidth <= 125000) return SX1278_LORA_BW_125KHZ;
    if(bandwidth <= 250000) return SX1278_LORA_BW_250KHZ;
    if(bandwidth <= 500000) return SX1278_LORA_BW_500KHZ;
    return SX1278_LORA_BW_250KHZ;
}

uint32_t calculate_crc32(const uint8_t *data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;
    if(data == NULL || length == 0){
        return 0;
    }

    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            } else {
                crc = (crc >> 1);
            }
        }
    }

    return crc;
}

void load_device_config() {
    DeviceConfig loaded_config;
    memcpy(&loaded_config, (void*) DEVICE_CONFIG_ADDRESS, sizeof(DeviceConfig));

    // Check crc
    uint32_t crc = calculate_crc32((uint8_t *) &loaded_config, sizeof(DeviceConfig) - sizeof(uint32_t));
    if(crc == loaded_config.crc32) {
        memcpy(&device_config, &loaded_config, sizeof(DeviceConfig));
    }
    else {
        indicate_error();
        write_config_to_flash(); // This overrides the config with the default values
    }
}


static uint32_t get_page(uint32_t Addr) // Single Bank Device
{
    return((Addr - FLASH_BASE) / FLASH_PAGE_SIZE);
}

// Function to write data to the flash memory
//  BYTE pointer, Size in BYTES, put what you want in a STRUCTURE

void write_flash(uint8_t *data, uint32_t size)
{
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t addr = DEVICE_CONFIG_ADDRESS; // 64-bit aligned
    uint64_t value;
    uint32_t PageError;

    size = (size + 7) & ~7; // Round to 64-bit words

    // Unlock the flash memory for writing
    HAL_FLASH_Unlock();

    /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

    /* Fill EraseInit structure, spanning size of write operation */
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = FLASH_BANK_1;
    EraseInitStruct.Page        = get_page(addr); // Starting Page
    EraseInitStruct.NbPages     = get_page(addr + size) - EraseInitStruct.Page + 1; // Page Count, at least 1

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        // Insert code to handle the error here

        HAL_FLASH_Lock();
        return;
    }

    size /= sizeof(uint64_t); // Byte count to 64-bit word count

    // Iterate over the array of data and write each element to the flash memory
    while(size--)
    {
        memcpy(&value, data, sizeof(uint64_t)); // alignment safe copy of byte array into 64-bit value

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, value) != HAL_OK)
        {
            // Insert code to handle the error here

            HAL_FLASH_Lock();
            return;
        }

        addr += sizeof(uint64_t); // Advance address
        data += sizeof(uint64_t); // Advance byte buffer
    }

    // Lock the flash memory after writing is complete
    HAL_FLASH_Lock();
}



void write_config_to_flash() {
    device_config.crc32 = calculate_crc32((uint8_t *) &device_config, sizeof(DeviceConfig) - sizeof(uint32_t));
    write_flash((uint8_t*) &device_config, sizeof(DeviceConfig));
}

bool set_lora_frequency(uint32_t frequency) {
    if(frequency < 410000000 || frequency > 525000000) return false;
    device_config.lora_frequency = frequency;
    write_config_to_flash();
    return true;
}

bool set_lora_tx_power(uint8_t tx_power) {
    if(tx_power < 1 || tx_power > 20) return false;
    device_config.lora_tx_power = tx_power;
    write_config_to_flash();
    return true;
}

bool set_lora_bandwidth(uint32_t bandwidth) {
    if(bandwidth < 7800 || bandwidth > 500000) return false;
    device_config.lora_bandwidth = bandwidth;
    write_config_to_flash();
    return true;
}

bool set_lora_sync_word(uint16_t sync_word) {
    device_config.lora_sync_word = sync_word;
    write_config_to_flash();
    return true;
}

bool set_lora_spreading_factor(uint8_t spreading_factor) {
    if(spreading_factor < 6 || spreading_factor > 12) return false;
    device_config.lora_spreading_factor = spreading_factor;
    write_config_to_flash();
    return true;
}



