#include <string.h>
#include <stdio.h>
#include "bugpack_data_manager.h"


BugPackDataStorageWrapper bugpack_data_wrappers[BUGPACK_DATA_STORAGE_SIZE] = {0};

BugPackData * get_bugpack_data(uint8_t bugpack_id) {
    for (int i = 0; i < BUGPACK_DATA_STORAGE_SIZE; i++) {
        if (bugpack_data_wrappers[i].bugpack_data.bugpack_id == bugpack_id) {
            return &bugpack_data_wrappers[i].bugpack_data;
        }
    }
    return NULL;
}

void set_bugpack_data(BugPackData *data) {
    for (int i = 0; i < BUGPACK_DATA_STORAGE_SIZE; i++) {
        if (bugpack_data_wrappers[i].bugpack_data.bugpack_id == data->bugpack_id) {
            memcpy(&bugpack_data_wrappers[i].bugpack_data, data, sizeof(BugPackData));
            bugpack_data_wrappers[i].recently_updated = true;
            bugpack_data_wrappers[i].last_updated_tick = HAL_GetTick();
            return;
        }
    }
    for (int i = 0; i < BUGPACK_DATA_STORAGE_SIZE; i++) {
        if (bugpack_data_wrappers[i].bugpack_data.bugpack_id == 0) {
            memcpy(&bugpack_data_wrappers[i].bugpack_data, data, sizeof(BugPackData));
            bugpack_data_wrappers[i].recently_updated = true;
            bugpack_data_wrappers[i].last_updated_tick = HAL_GetTick();
            return;
        }
    }
}

char* bugpack_data_to_string(BugPackData* data) {
    static char str[256]; // Buffer to hold the string. Note: this is not thread-safe!
    sprintf(str, "%d,%hu,%.4f,%.4f,%.4f",
            data->bugpack_id, data->battery_voltage, data->acceleration_x, data->acceleration_y, data->acceleration_z);
    return str;
}

void get_all_data_as_a_string(char * buffer) {
    for (int i = 0; i < BUGPACK_DATA_STORAGE_SIZE; i++) {
        if (bugpack_data_wrappers[i].recently_updated && bugpack_data_wrappers[i].bugpack_data.bugpack_id != 0) {
            char temp[256]; // Assuming that this size is enough to hold one record
            // Assuming that BugPackData has a method to convert its data to string
            sprintf(temp, "%s\n", bugpack_data_to_string(&bugpack_data_wrappers[i].bugpack_data));
            strcat(buffer, temp);
            bugpack_data_wrappers[i].recently_updated = false;
        }
    }
}