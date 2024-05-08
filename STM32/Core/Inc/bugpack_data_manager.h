#ifndef STM32_BUGPACK_DATA_MANAGER_H
#define STM32_BUGPACK_DATA_MANAGER_H

#include <stdbool.h>
#include "main.h"

#define BUGPACK_DATA_STORAGE_SIZE 8

typedef struct BugPackData {
    uint8_t bugpack_id;
    uint16_t battery_voltage;
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
} BugPackData;

typedef struct BugPackDataStorageWrapper {
    BugPackData bugpack_data;
    bool recently_updated;
    uint64_t last_updated_tick;
} BugPackDataStorageWrapper;

void get_all_data_as_a_string(char * buffer);

void set_bugpack_data(BugPackData *data);

#endif //STM32_BUGPACK_DATA_MANAGER_H
