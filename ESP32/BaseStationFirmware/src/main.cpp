#include <Arduino.h>
//#include <esp_task_wdt.h>
#include "lora_handler.h"
#include "debug.h"
#include "task_monitoring.h"
#include "display_handler.h"
#include "wifi_handler.h"


// Prepare semaphores etc
xSemaphoreHandle i2c_gatekeeper = xSemaphoreCreateMutex();


// Prepare tasks
[[noreturn]]
void lora_task(void * parameters) {
    register_task_monitoring("loraTask");
    for(;;) {
        check_lora_receiver();
        delay(10); // Prevent starvation of other tasks.
    }
}

[[noreturn]]
void display_task(void * parameters) {
    register_task_monitoring("displayTask");
    for(;;) {
        display_update();
        delay(150);
    }
}

[[noreturn]]
void wifi_task(void * parameters) {
    unsigned int timer = millis();
    register_task_monitoring("wifiTask");
    for(;;) {
        keep_wifi_alive();    // Delays inside the function
        if(millis() - timer > 10000) {
            confirm_task_active("wifiTask");
            timer = millis();
        }
    }
}

[[noreturn]]
void upload_task(void * parameters) {
    unsigned int timer = millis();
    register_task_monitoring("uploadTaks");
    for(;;) {
        handle_unprocessed_packets();
        delay(10);
        if(millis() - timer > 10000) {
            confirm_task_active("uploadTask");
            timer = millis();
        }
    }
}


void setup() {

    // Initialize the different modules
    Serial.begin(115200);
    init_display();
    lora_init();
//    task_monitor_init();

    // Startup the tasks
    xTaskCreate(
            lora_task, // Function that is called
            "LoRa Task", //Debug name of the task
            16384,  // Stack size in bytes
            nullptr, // Task parameters
            15,  // Task priority
            nullptr // Task handle
    );
    delay(5);
    xTaskCreate(
            display_task, // Function that is called
            "display_task", //Debug name of the task
            2048,  // Stack size in bytes
            nullptr, // Task parameters
            5,  // Task priority
            nullptr // Task handle
    );
    delay(5);
    xTaskCreate(
            wifi_task, // Function that is called
            "wifi_task", //Debug name of the task
            4096,  // Stack size in bytes
            nullptr, // Task parameters
            5,  // Task priority
            nullptr // Task handle
    );
    delay(5);
    xTaskCreate(
            upload_task, // Function that is called
            "upload_task", //Debug name of the task
            8192,  // Stack size in bytes
            nullptr, // Task parameters
            5,  // Task priority
            nullptr // Task handle
    );
    delay(5);


    debugPrintln("Device initialization complete!");
}

void loop() {
    // offloaded to tasks
    delay(1000);
//    esp_task_wdt_reset();
    debugPrintln("Looping");
}