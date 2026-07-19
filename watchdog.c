/**
 * @file watchdog.c
 * @brief System Watchdog & Safety Monitor
 * @version 2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "watchdog.h"
#include "hal.h"
#include "traffic_system.h"

/* ============================================================================
 * INTERNAL STATE
 * ============================================================================ */
static WdgTaskEntry wdg_tasks[WDG_MAX_MONITORED_TASKS];
static SystemHealth system_health = {0};
static FaultRecord fault_log[MAX_FAULT_RECORDS];
static uint8_t fault_count = 0;
static uint8_t fault_index = 0;
static uint32_t wdg_start_time = 0;
static uint32_t consecutive_faults = 0;

/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */
int wdg_init(uint32_t timeout_ms) {
    memset(wdg_tasks, 0, sizeof(wdg_tasks));
    memset(&system_health, 0, sizeof(system_health));
    memset(fault_log, 0, sizeof(fault_log));
    fault_count = 0;
    fault_index = 0;
    consecutive_faults = 0;
    wdg_start_time = hal_get_tick_ms();

    system_health.healthy = true;
    system_health.uptime_seconds = 0;
    system_health.reset_count = 0;
    system_health.watchdog_resets = 0;
    system_health.brownout_resets = 0;
    system_health.software_resets = 0;
    system_health.cpu_temperature = 25.0f;
    system_health.supply_voltage = 12.0f;
    system_health.supply_current = 0.5f;
    system_health.flash_errors = 0;
    system_health.ram_errors = 0;
    system_health.comm_errors = 0;

    hal_wdg_init(timeout_ms);

    return 0;
}

/* ============================================================================
 * TASK REGISTRATION
 * ============================================================================ */
int wdg_register_task(uint8_t task_id, const char *name, uint32_t period_ms, bool critical) {
    for (int i = 0; i < WDG_MAX_MONITORED_TASKS; i++) {
        if (!wdg_tasks[i].active) {
            wdg_tasks[i].task_id = task_id;
            strncpy(wdg_tasks[i].name, name, sizeof(wdg_tasks[i].name) - 1);
            wdg_tasks[i].expected_period_ms = period_ms;
            wdg_tasks[i].last_pet_time_ms = hal_get_tick_ms();
            wdg_tasks[i].miss_count = 0;
            wdg_tasks[i].active = true;
            wdg_tasks[i].critical = critical;
            return 0;
        }
    }
    return -1;
}

/* ============================================================================
 * PET (REFRESH)
 * ============================================================================ */
int wdg_pet(uint8_t task_id) {
    for (int i = 0; i < WDG_MAX_MONITORED_TASKS; i++) {
        if (wdg_tasks[i].active && wdg_tasks[i].task_id == task_id) {
            wdg_tasks[i].last_pet_time_ms = hal_get_tick_ms();
            wdg_tasks[i].miss_count = 0;
            return 0;
        }
    }
    return -1;
}

/* ============================================================================
 * HEALTH CHECK
 * ============================================================================ */
int wdg_check_all(void) {
    uint32_t now = hal_get_tick_ms();
    bool all_healthy = true;

    for (int i = 0; i < WDG_MAX_MONITORED_TASKS; i++) {
        if (!wdg_tasks[i].active) continue;

        uint32_t elapsed = now - wdg_tasks[i].last_pet_time_ms;

        if (elapsed > wdg_tasks[i].expected_period_ms * 3) {
            wdg_tasks[i].miss_count++;

            if (wdg_tasks[i].critical && wdg_tasks[i].miss_count >= 3) {
                wdg_record_fault(FAULT_WATCHDOG_TIMEOUT, wdg_tasks[i].task_id,
                                elapsed, wdg_tasks[i].name);
                all_healthy = false;
                consecutive_faults++;

                if (consecutive_faults >= WDG_MAX_FAULTS_BEFORE_RESET) {
                    wdg_force_reset("Critical task watchdog timeout");
                    return -1;
                }
            }
        }
    }

    system_health.healthy = all_healthy;
    system_health.uptime_seconds = (now - wdg_start_time) / 1000;

    if (all_healthy) {
        consecutive_faults = 0;
        hal_wdg_refresh();
    }

    return all_healthy ? 0 : -1;
}

/* ============================================================================
 * FAULT RECORDING
 * ============================================================================ */
int wdg_record_fault(FaultType type, uint8_t task_id, uint32_t error_code, const char *desc) {
    FaultRecord *record = &fault_log[fault_index];

    record->type = type;
    record->timestamp = hal_get_tick_ms();
    record->task_id = task_id;
    record->error_code = error_code;

    if (desc != NULL) {
        strncpy(record->description, desc, sizeof(record->description) - 1);
    } else {
        record->description[0] = '\0';
    }

    fault_index = (fault_index + 1) % MAX_FAULT_RECORDS;
    if (fault_count < MAX_FAULT_RECORDS) {
        fault_count++;
    }

    /* Update health counters */
    switch (type) {
        case FAULT_SENSOR_FAILURE:
            /* Sensor failures are recorded in the fault log; they are not communication errors. */
            break;
        case FAULT_MEMORY_CORRUPTION:
            system_health.ram_errors++;
            break;
        case FAULT_OVER_TEMPERATURE:
            system_health.cpu_temperature = (float)error_code;
            break;
        case FAULT_POWER_LOW:
            system_health.supply_voltage = (float)error_code / 1000.0f;
            break;
        default:
            break;
    }

    return 0;
}

/* ============================================================================
 * FAULT QUERY
 * ============================================================================ */
void wdg_get_last_faults(FaultRecord *records, uint8_t *count) {
    if (records == NULL || count == NULL) return;

    uint8_t n = (*count < fault_count) ? *count : fault_count;
    uint8_t start = (fault_index + MAX_FAULT_RECORDS - n) % MAX_FAULT_RECORDS;

    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (start + i) % MAX_FAULT_RECORDS;
        memcpy(&records[i], &fault_log[idx], sizeof(FaultRecord));
    }

    *count = n;
}

void wdg_print_fault_log(void) {
    printf("\n=== FAULT LOG (%u entries) ===\n", fault_count);
    printf("Time      Type                    Task  Code    Description\n");
    printf("--------- ----------------------- ----- ------- ------------------------------\n");

    for (uint8_t i = 0; i < fault_count && i < 20; i++) {
        uint8_t idx = (fault_index + MAX_FAULT_RECORDS - 1 - i) % MAX_FAULT_RECORDS;
        FaultRecord *r = &fault_log[idx];

        const char *type_str = "UNKNOWN";
        switch (r->type) {
            case FAULT_NONE: type_str = "NONE"; break;
            case FAULT_WATCHDOG_TIMEOUT: type_str = "WDG_TIMEOUT"; break;
            case FAULT_STACK_OVERFLOW: type_str = "STACK_OVF"; break;
            case FAULT_MEMORY_CORRUPTION: type_str = "MEM_CORRUPT"; break;
            case FAULT_SENSOR_FAILURE: type_str = "SENSOR_FAIL"; break;
            case FAULT_COMM_TIMEOUT: type_str = "COMM_TIMEOUT"; break;
            case FAULT_POWER_LOW: type_str = "POWER_LOW"; break;
            case FAULT_OVER_TEMPERATURE: type_str = "OVER_TEMP"; break;
            case FAULT_STUCK_LIGHT: type_str = "STUCK_LIGHT"; break;
            case FAULT_INVALID_STATE: type_str = "INVALID_STATE"; break;
        }

        printf("%-9u %-23s %-5u %-7u %s\n",
               r->timestamp, type_str, r->task_id, r->error_code,
               r->description[0] ? r->description : "-");
    }
}

/* ============================================================================
 * HEALTH STATUS
 * ============================================================================ */
void wdg_get_health(SystemHealth *health) {
    if (health == NULL) return;
    memcpy(health, &system_health, sizeof(SystemHealth));
}

bool wdg_is_healthy(void) {
    return system_health.healthy;
}

/* ============================================================================
 * FORCE RESET
 * ============================================================================ */
void wdg_force_reset(const char *reason) {
    printf("\n[WDG] FORCED RESET: %s\n", reason ? reason : "Unknown");
    system_health.watchdog_resets++;
    system_health.reset_count++;

    /* Log final fault */
    wdg_record_fault(FAULT_WATCHDOG_TIMEOUT, 0xFF, 0, reason);

    hal_delay_ms(100);
    hal_system_reset();
}
