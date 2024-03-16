#ifndef STM32_INDICATOR_UTILS_H
#define STM32_INDICATOR_UTILS_H

void indicate_startup();

enum buzzer_state_t {
    BUZZER_OFF,
    FATAL_ERROR,
    BUZZER_STATE_MAX_VAL
};

enum led_state_t {
    LED_OFF,
    FATAL_ERROR_LED,
    LED_STATE_MAX_VAL
};

void process_indication_state();

#endif //STM32_INDICATOR_UTILS_H
