/**
 * @file scheduler.c
 * @brief Cooperative Task Scheduler Implementation
 * @version 2.0
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include "scheduler.h"
#include "hal.h"

/* ============================================================================
 * INTERNAL STATE
 * ============================================================================ */
static TaskControlBlock tasks[MAX_TASKS];
static uint32_t task_count = 0;
static SchedulerStats stats = {0};
static volatile bool scheduler_running = false;
static uint32_t scheduler_start_ms = 0;

/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */
int scheduler_init(void) {
    memset(tasks, 0, sizeof(tasks));
    task_count = 0;
    memset(&stats, 0, sizeof(stats));
    scheduler_running = false;
    scheduler_start_ms = hal_get_tick_ms();

    /* Create idle task */
    tasks[0].id = IDLE_TASK_ID;
    strncpy(tasks[0].name, "idle", TASK_NAME_LEN - 1);
    tasks[0].func = NULL;
    tasks[0].state = TASK_READY;
    tasks[0].priority = PRIO_IDLE;
    tasks[0].periodic = true;
    tasks[0].period_ms = 10;
    task_count = 1;

    return 0;
}

/* ============================================================================
 * TASK MANAGEMENT
 * ============================================================================ */
int scheduler_create_task(const char *name, TaskFunc func, void *arg,
                          TaskPriority priority, uint32_t period_ms, bool periodic) {
    if (task_count >= MAX_TASKS) return -1;
    if (func == NULL) return -1;

    uint8_t id = (uint8_t)task_count;
    TaskControlBlock *task = &tasks[id];

    task->id = id;
    strncpy(task->name, name, TASK_NAME_LEN - 1);
    task->func = func;
    task->arg = arg;
    task->state = TASK_READY;
    task->priority = priority;
    task->period_ms = period_ms;
    task->last_run_ms = hal_get_tick_ms();
    task->exec_time_us = 0;
    task->exec_count = 0;
    task->deadline_miss_count = 0;
    task->periodic = periodic;

    task_count++;
    stats.total_tasks = task_count;
    stats.active_tasks++;

    return id;
}

int scheduler_delete_task(uint8_t task_id) {
    if (task_id >= task_count || task_id == IDLE_TASK_ID) return -1;

    tasks[task_id].state = TASK_TERMINATED;
    tasks[task_id].func = NULL;
    stats.active_tasks--;

    return 0;
}

int scheduler_suspend_task(uint8_t task_id) {
    if (task_id >= task_count) return -1;
    if (tasks[task_id].state == TASK_TERMINATED) return -1;

    tasks[task_id].state = TASK_SUSPENDED;
    return 0;
}

int scheduler_resume_task(uint8_t task_id) {
    if (task_id >= task_count) return -1;
    if (tasks[task_id].state == TASK_TERMINATED) return -1;

    tasks[task_id].state = TASK_READY;
    return 0;
}

TaskControlBlock* scheduler_get_task(uint8_t task_id) {
    if (task_id >= task_count) return NULL;
    return &tasks[task_id];
}

/* ============================================================================
 * SCHEDULER CORE
 * ============================================================================ */
void scheduler_run(void) {
    scheduler_running = true;

    while (scheduler_running) {
        uint32_t now = hal_get_tick_ms();
        uint32_t highest_priority = PRIO_IDLE;
        int selected_task = -1;

        /* Find highest priority ready task */
        for (uint32_t i = 1; i < task_count; i++) {
            if (tasks[i].state != TASK_READY && tasks[i].state != TASK_RUNNING) {
                continue;
            }

            /* Check if periodic task is due */
            if (tasks[i].periodic) {
                if ((now - tasks[i].last_run_ms) < tasks[i].period_ms) {
                    continue;
                }
            }

            if ((uint32_t)tasks[i].priority > highest_priority) {
                highest_priority = tasks[i].priority;
                selected_task = (int)i;
            }
        }

        /* Run selected task */
        if (selected_task >= 0) {
            TaskControlBlock *task = &tasks[selected_task];

            struct timeval start, end;
            gettimeofday(&start, NULL);

            task->state = TASK_RUNNING;
            task->func(task->arg);
            task->state = TASK_READY;
            task->last_run_ms = now;
            task->exec_count++;

            gettimeofday(&end, NULL);
            uint32_t exec_us = (uint32_t)((end.tv_sec - start.tv_sec) * 1000000 +
                                          (end.tv_usec - start.tv_usec));
            task->exec_time_us += exec_us;

            stats.context_switches++;
            stats.total_exec_time_us += exec_us;

            /* Check deadline miss */
            if (task->periodic && exec_us > task->period_ms * 1000) {
                task->deadline_miss_count++;
                stats.deadline_misses++;
            }
        } else {
            /* Idle task */
            stats.idle_time_us += 1000;
            hal_delay_ms(1);
        }
    }
}

void scheduler_tick(void) {
    /* Called from SysTick ISR - mark tasks as ready if due */
    uint32_t now = hal_get_tick_ms();

    for (uint32_t i = 1; i < task_count; i++) {
        if (tasks[i].state == TASK_BLOCKED || tasks[i].state == TASK_SUSPENDED) {
            continue;
        }

        if (tasks[i].periodic && (now - tasks[i].last_run_ms) >= tasks[i].period_ms) {
            tasks[i].state = TASK_READY;
        }
    }
}

void scheduler_yield(void) {
    /* Cooperative yield - return to scheduler */
}

void scheduler_get_stats(SchedulerStats *s) {
    if (s == NULL) return;
    memcpy(s, &stats, sizeof(SchedulerStats));
}

void scheduler_print_tasks(void) {
    printf("\n=== TASK TABLE ===\n");
    printf("ID  Name          State    Prio Period  ExecCount Misses\n");
    printf("--- ------------- -------- ---- ------- --------- ------\n");

    for (uint32_t i = 0; i < task_count; i++) {
        const char *state_str = "UNKNOWN";
        switch (tasks[i].state) {
            case TASK_READY: state_str = "READY"; break;
            case TASK_RUNNING: state_str = "RUNNING"; break;
            case TASK_BLOCKED: state_str = "BLOCKED"; break;
            case TASK_SUSPENDED: state_str = "SUSPENDED"; break;
            case TASK_TERMINATED: state_str = "TERM"; break;
        }

        printf("%-3u %-13s %-8s %-4u %-7u %-9u %u\n",
               tasks[i].id,
               tasks[i].name,
               state_str,
               tasks[i].priority,
               tasks[i].period_ms,
               tasks[i].exec_count,
               tasks[i].deadline_miss_count);
    }
}

/* ============================================================================
 * MUTEX
 * ============================================================================ */
int mutex_init(SimpleMutex *mutex) {
    if (mutex == NULL) return -1;
    mutex->locked = false;
    mutex->owner = 0xFF;
    mutex->lock_count = 0;
    return 0;
}

int mutex_lock(SimpleMutex *mutex, uint32_t timeout_ms) {
    if (mutex == NULL) return -1;

    uint32_t start = hal_get_tick_ms();
    while (mutex->locked) {
        if (timeout_ms > 0 && (hal_get_tick_ms() - start) > timeout_ms) {
            return -1; /* Timeout */
        }
        hal_delay_ms(1);
    }

    mutex->locked = true;
    mutex->owner = 0; /* Would be task ID in real RTOS */
    mutex->lock_count++;
    return 0;
}

int mutex_unlock(SimpleMutex *mutex) {
    if (mutex == NULL) return -1;
    mutex->locked = false;
    mutex->owner = 0xFF;
    return 0;
}

/* ============================================================================
 * EVENT FLAGS
 * ============================================================================ */
int event_init(EventFlags *event) {
    if (event == NULL) return -1;
    event->flags = 0;
    event->waiting_task = 0xFF;
    return 0;
}

int event_set(EventFlags *event, uint32_t flags) {
    if (event == NULL) return -1;
    event->flags |= flags;
    return 0;
}

int event_wait(EventFlags *event, uint32_t flags, uint32_t timeout_ms) {
    if (event == NULL) return -1;

    uint32_t start = hal_get_tick_ms();
    while ((event->flags & flags) != flags) {
        if (timeout_ms > 0 && (hal_get_tick_ms() - start) > timeout_ms) {
            return -1;
        }
        hal_delay_ms(1);
    }
    return 0;
}

int event_clear(EventFlags *event, uint32_t flags) {
    if (event == NULL) return -1;
    event->flags &= ~flags;
    return 0;
}

/* ============================================================================
 * MESSAGE QUEUE
 * ============================================================================ */
int queue_init(MsgQueue *queue, uint16_t item_size) {
    if (queue == NULL || item_size == 0 || item_size > MSG_QUEUE_SIZE) return -1;
    memset(queue->buffer, 0, sizeof(queue->buffer));
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->size = item_size;
    mutex_init(&queue->lock);
    return 0;
}

int queue_send(MsgQueue *queue, const void *item, uint32_t timeout_ms) {
    if (queue == NULL || item == NULL) return -1;

    if (mutex_lock(&queue->lock, timeout_ms) != 0) return -1;

    if (queue->count >= MSG_QUEUE_SIZE / queue->size) {
        mutex_unlock(&queue->lock);
        return -1; /* Full */
    }

    uint16_t idx = queue->tail * queue->size;
    memcpy(&queue->buffer[idx], item, queue->size);
    queue->tail = (queue->tail + 1) % (MSG_QUEUE_SIZE / queue->size);
    queue->count++;

    mutex_unlock(&queue->lock);
    return 0;
}

int queue_receive(MsgQueue *queue, void *item, uint32_t timeout_ms) {
    if (queue == NULL || item == NULL) return -1;

    uint32_t start = hal_get_tick_ms();

    while (queue->count == 0) {
        if (timeout_ms > 0 && (hal_get_tick_ms() - start) > timeout_ms) {
            return -1;
        }
        hal_delay_ms(1);
    }

    if (mutex_lock(&queue->lock, timeout_ms) != 0) return -1;

    uint16_t idx = queue->head * queue->size;
    memcpy(item, &queue->buffer[idx], queue->size);
    queue->head = (queue->head + 1) % (MSG_QUEUE_SIZE / queue->size);
    queue->count--;

    mutex_unlock(&queue->lock);
    return 0;
}

int queue_count(const MsgQueue *queue) {
    if (queue == NULL) return -1;
    return (int)queue->count;
}
