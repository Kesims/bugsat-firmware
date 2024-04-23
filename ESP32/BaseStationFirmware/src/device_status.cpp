#include "Arduino.h"
#include "device_status.h"

float get_battery_voltage() {
    return (float) (analogReadMilliVolts(35))/500;
}

byte get_battery_level() {
    float voltage = get_battery_voltage();
    if(voltage > 3.95) return 4;
    if(voltage > 3.85) return 3;
    if(voltage > 3.75) return 2;
    if(voltage > 3.65) return 1;
    return 0;
}