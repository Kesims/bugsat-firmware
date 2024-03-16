#include <stdbool.h>
#include "indicator_utils.h"
#include "main.h"
#include "stm32l4xx_hal.h"

extern TIM_HandleTypeDef htim3; // When turned on, buzzer will sound

void indicate_startup() {
    // do 2 short beeps
    for (int i = 0; i < 2; i++) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_Delay(200);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_Delay(100);
    }
}

volatile enum buzzer_state_t buzzer_state = BUZZER_OFF;
volatile bool buzzer_state_changed = false;
volatile enum led_state_t led_state = LED_OFF;
volatile bool led_state_changed = false;

void set_buzzer_state(enum buzzer_state_t state) {
    buzzer_state = state;
    buzzer_state_changed = true;
}

void set_led_state(enum led_state_t state) {
    led_state = state;
    led_state_changed = true;
}

void process_indication_state() {
    //TODO: implement, if needed. Not necessary for now.
}