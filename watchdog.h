/**
 * @file watchdog.h
 * @brief System Watchdog & Safety Monitor
 * @version 2.0
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WDG_MAX_MONITORED_TASKS     8
#define WDG_CHECK_INTERVAL_MS       100
#define WDG_TIMEOUT_MS              1000
#define WDG_MAX_FAULTS_BEFORE_RESET 3

/* Watchdog Task Registration */
typedef struct {
    uint8_t task_id;
    char name[16];
    uint32_t expected_period_ms;
    uint32_t last_pet_time_ms;
    uint32_t miss_count;
    bool active;
    bool critical;
} WdgTaskEntry;

/* System Health Status */
typedef struct {
    bool healthy;
    uint32_t uptime_seconds;
    uint32_t reset_count;
    uint32_t watchdog_resets;
    uint32_t brownout_resets;
    uint32_t software_resets;
    float cpu_temperature;
    float supply_voltage;
    float supply_current;
    uint32_t flash_errors;
    uint32_t ram_errors;
    uint32_t comm_errors;
} SystemHealth;

/* Fault Types */
typedef enum {
    FAULT_NONE = 0,
    FAULT_WATCHDOG_TIMEOUT,
    FAULT_STACK_OVERFLOW,
    FAULT_MEMORY_CORRUPTION,
    FAULT_SENSOR_FAILURE,
    FAULT_COMM_TIMEOUT,
    FAULT_POWER_LOW,
    FAULT_OVER_TEMPERATURE,
    FAULT_STUCK_LIGHT,
    FAULT_INVALID_STATE
} FaultType;

/* Fault Record */
typedef struct {
    FaultType type;
    uint32_t timestamp;
    uint8_t task_id;
    uint32_t error_code;
    char description[64];
} FaultRecord;

#define MAX_FAULT_RECORDS   32

/* ============================================================================
 * API
 * ============================================================================ */
int wdg_init(uint32_t timeout_ms);
int wdg_register_task(uint8_t task_id, const char *name, uint32_t period_ms, bool critical);
int wdg_pet(uint8_t task_id);
int wdg_check_all(void);
void wdg_force_reset(const char *reason);
void wdg_get_health(SystemHealth *health);
int wdg_record_fault(FaultType type, uint8_t task_id, uint32_t error_code, const char *desc);
void wdg_get_last_faults(FaultRecord *records, uint8_t *count);
void wdg_print_fault_log(void);
bool wdg_is_healthy(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
