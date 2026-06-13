/**
 * @file scheduler.h
 * @brief Cooperative Task Scheduler (RTOS-like for bare metal)
 * @version 2.0
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TASKS           16
#define TASK_NAME_LEN       16
#define IDLE_TASK_ID        0

/* Task States */
typedef enum {
    TASK_READY = 0,
    TASK_RUNNING,
    TASK_BLOCKED,
    TASK_SUSPENDED,
    TASK_TERMINATED
} TaskState;

/* Task Priorities */
typedef enum {
    PRIO_IDLE = 0,
    PRIO_LOW = 1,
    PRIO_NORMAL = 2,
    PRIO_HIGH = 3,
    PRIO_CRITICAL = 4,
    PRIO_ISR = 5
} TaskPriority;

/* Task Function Type */
typedef void (*TaskFunc)(void *arg);

/* Task Control Block */
typedef struct {
    uint8_t id;
    char name[TASK_NAME_LEN];
    TaskFunc func;
    void *arg;
    TaskState state;
    TaskPriority priority;
    uint32_t period_ms;
    uint32_t last_run_ms;
    uint32_t exec_time_us;
    uint32_t exec_count;
    uint32_t deadline_miss_count;
    bool periodic;
} TaskControlBlock;

/* Scheduler Stats */
typedef struct {
    uint32_t total_tasks;
    uint32_t active_tasks;
    uint32_t context_switches;
    uint32_t total_exec_time_us;
    uint32_t idle_time_us;
    uint32_t deadline_misses;
    uint32_t max_latency_us;
    uint32_t avg_latency_us;
} SchedulerStats;

/* ============================================================================
 * API
 * ============================================================================ */
int scheduler_init(void);
int scheduler_create_task(const char *name, TaskFunc func, void *arg,
                          TaskPriority priority, uint32_t period_ms, bool periodic);
int scheduler_delete_task(uint8_t task_id);
int scheduler_suspend_task(uint8_t task_id);
int scheduler_resume_task(uint8_t task_id);
void scheduler_run(void);
void scheduler_tick(void);  /* Call from SysTick ISR */
void scheduler_yield(void);
TaskControlBlock* scheduler_get_task(uint8_t task_id);
void scheduler_get_stats(SchedulerStats *stats);
void scheduler_print_tasks(void);

/* Mutex (simple binary semaphore) */
typedef struct {
    volatile bool locked;
    uint8_t owner;
    uint32_t lock_count;
} SimpleMutex;

int mutex_init(SimpleMutex *mutex);
int mutex_lock(SimpleMutex *mutex, uint32_t timeout_ms);
int mutex_unlock(SimpleMutex *mutex);

/* Event Flags */
typedef struct {
    volatile uint32_t flags;
    uint8_t waiting_task;
} EventFlags;

int event_init(EventFlags *event);
int event_set(EventFlags *event, uint32_t flags);
int event_wait(EventFlags *event, uint32_t flags, uint32_t timeout_ms);
int event_clear(EventFlags *event, uint32_t flags);

/* Message Queue */
#define MSG_QUEUE_SIZE  32

typedef struct {
    uint8_t buffer[MSG_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    uint16_t size;
    SimpleMutex lock;
} MsgQueue;

int queue_init(MsgQueue *queue, uint16_t item_size);
int queue_send(MsgQueue *queue, const void *item, uint32_t timeout_ms);
int queue_receive(MsgQueue *queue, void *item, uint32_t timeout_ms);
int queue_count(const MsgQueue *queue);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
