#ifndef STM32_DEBUG_PRINTF_H
#define STM32_DEBUG_PRINTF_H

#include <stdio.h>
#include "main.h"

#define UART_DEBUG ENABLE_DEBUG_MESSAGES

#if UART_DEBUG == 1
#define debugPrint(x) printf(x)
#define debugPrintf(x, ...) printf(x, __VA_ARGS__)
#else
#define debugPrint(x)
#define debugPrintf(x, ...)
#endif

#endif //STM32_DEBUG_PRINTF_H
