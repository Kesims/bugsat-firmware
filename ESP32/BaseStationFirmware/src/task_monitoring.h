#ifndef BASESTATIONFIRMWARE_TASK_MONITORING_H
#define BASESTATIONFIRMWARE_TASK_MONITORING_H

void task_monitor_init();
void register_task_monitoring(String taskName);
void unregister_task_monitoring(const String& taskName);
void confirm_task_active(String taskName);
void reset_all_tasks_status();
void try_watchdog_reset();
void debug_print_monitoring_status();

#endif //BASESTATIONFIRMWARE_TASK_MONITORING_H
