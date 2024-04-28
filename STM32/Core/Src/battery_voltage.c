#include "battery_voltage.h"

extern ADC_HandleTypeDef hadc1;

uint16_t get_battery_voltage() {
    uint16_t adc_value = 0;
    // Start ADC
    HAL_ADC_Start(&hadc1);

    // Poll for conversion completion
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        // If the conversion is completed, read the ADC value
        adc_value = HAL_ADC_GetValue(&hadc1);
    }

    // Stop ADC
    HAL_ADC_Stop(&hadc1);

    // Convert the ADC value to voltage (in mV)
    uint32_t voltage = adc_value * 3300 / 4095;

    return voltage;
}
