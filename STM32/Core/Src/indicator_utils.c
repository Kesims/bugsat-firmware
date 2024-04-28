#include <stdbool.h>
#include "indicator_utils.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim3; // When turned on, buzzer will sound

void indicate_startup() {
    // do 2 short beeps
    for (int i = 0; i < 2; i++) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        osDelay(200);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        osDelay(100);
    }
}

void indicate_error() {
    // set red LED on
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    // do 3 long beeps
    for (int i = 0; i < 3; i++) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        osDelay(500);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        osDelay(200);
    }
}