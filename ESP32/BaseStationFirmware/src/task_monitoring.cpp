#include <Arduino.h>
#include <Dictionary.h>
#include <esp_task_wdt.h>
#include "debug.h"
#include "task_monitoring.h"

//Watchdog timeout in seconds
#define WDT_TIMEOUT 33     // Not expecting any problems, but if anything bad happens, half a minute is still kinda OK to be down in worst case


Dictionary &taskList = *(new Dictionary);

void task_monitor_init() {
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(nullptr);
    esp_task_wdt_reset();
}

void register_task_monitoring(String taskName) {
    taskList.insert(taskName, true);
}

void unregister_task_monitoring(const String& taskName) {
    taskList.remove(taskName);
}

void confirm_task_active(String taskName) {
    taskList.insert(std::move(taskName), true);
}

void reset_all_tasks_status() {
    unsigned int cnt = taskList.count();
    for (int i=0; i < cnt; i++) {
        taskList.insert(taskList(i), false);
    }
}

void try_watchdog_reset() {
    unsigned int cnt = taskList.count();
    for (int i=0; i < cnt; i++) {
//        debugPrintln(taskList[taskList(i)]);
        if(!(taskList[taskList(i)].toInt())) return; // If any of the tasks status is false
    }
    esp_task_wdt_reset();
    reset_all_tasks_status();
}

void debug_print_monitoring_status() {
    unsigned int cnt = taskList.count();
    for (int i=0; i < cnt; i++) {
        debugPrintln(taskList(i) + ": " + String(taskList[taskList(i)]));
    }
}
