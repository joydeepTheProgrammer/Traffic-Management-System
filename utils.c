#include <pthread.h>
/**
 * @file utils.c
 * @brief Traffic System Utilities & Global Definitions
 * @version 2.0
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/time.h>
#include "traffic_system.h"

/* ============================================================================
 * GLOBAL INSTANCES
 * ============================================================================ */
TrafficSystem g_system;
EmergencyAlert g_emergency_alerts[MAX_EMERGENCY_VEHICLES];
volatile bool g_running = true;

/* ============================================================================
 * TIMESTAMP
 * ============================================================================ */
void get_timestamp_str(char *buf, size_t len) {
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", tm_info);
}

/* ============================================================================
 * LOGGING
 * ============================================================================ */
static pthread_mutex_t g_log_mutex = PTHREAD_MUTEX_INITIALIZER;

void log_event(const char *format, ...) {
    char timestamp[32];
    char buffer[LOG_BUFFER_SIZE];
    va_list args;

    get_timestamp_str(timestamp, sizeof(timestamp));
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    pthread_mutex_lock(&g_log_mutex);
    printf("[%s] %s\n", timestamp, buffer);
    fflush(stdout);
    pthread_mutex_unlock(&g_log_mutex);
}

/* ============================================================================
 * STRING CONVERSIONS
 * ============================================================================ */
const char* direction_to_str(Direction d) {
    static const char *names[] = {"NORTH", "SOUTH", "EAST", "WEST"};
    return (d < NUM_DIRECTIONS) ? names[d] : "UNKNOWN";
}

const char* light_to_str(TrafficLightState s) {
    static const char *names[] = {"RED", "YELLOW", "GREEN", "BLINK-YELLOW", "OFF"};
    return (s <= LIGHT_OFF) ? names[s] : "UNKNOWN";
}

const char* mode_to_str(SystemMode m) {
    static const char *names[] = {"NORMAL", "EMERGENCY", "PEDESTRIAN", "CONGESTION", "NIGHT", "FAULT"};
    return (m <= MODE_FAULT) ? names[m] : "UNKNOWN";
}

/* ============================================================================
 * CALCULATIONS
 * ============================================================================ */
float calculate_density(LaneController *lane) {
    if (lane == NULL || lane->vehicle_count == 0) return 0.0f;
    float density = (float)lane->vehicle_count / MAX_QUEUE_LENGTH;
    return (density > 1.0f) ? 1.0f : density;
}

float calculate_moving_average(LaneController *lane) {
    if (lane == NULL) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        sum += lane->density_history[i];
    }
    return sum / HISTORY_SIZE;
}
