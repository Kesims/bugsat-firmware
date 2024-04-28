#ifndef STM32_SENSORS_HANDLER_H
#define STM32_SENSORS_HANDLER_H

#include "main.h"

typedef struct BPM390BufferData {
    float temperature;
    float pressure;
} BPM390BufferData;

typedef struct LIS3DHBufferData {
    float accelerationX;
    float accelerationY;
    float accelerationZ;
} LIS3DHBufferData;

void sensors_init();

_Noreturn void sensors_task_work();

#endif //STM32_SENSORS_HANDLER_H
