/**
 * @file traffic_system.h
 * @brief Traffic Management System - Core Definitions
 * @version 2.0
 * @date 2026-06-13
 * 
 * Target: ARM Cortex-M4 / POSIX / FreeRTOS
 */

#ifndef TRAFFIC_SYSTEM_H
#define TRAFFIC_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */
#define NUM_DIRECTIONS          4
#define NUM_LANES               4
#define MAX_SENSORS_PER_LANE    3
#define MAX_QUEUE_LENGTH        50
#define HISTORY_SIZE            10
#define MAX_EMERGENCY_VEHICLES  5
#define LOG_BUFFER_SIZE         256

#define MIN_GREEN_TIME          10
#define MAX_GREEN_TIME          120
#define YELLOW_TIME             3
#define ALL_RED_TIME            2
#define PEDESTRIAN_WALK_TIME    15
#define SENSOR_SAMPLE_RATE_MS   500
#define SYSTEM_TICK_MS          100
#define CONGESTION_THRESHOLD    85      /* Percentage */

#define EMERGENCY_PRIORITY      255
#define PEDESTRIAN_PRIORITY     200
#define NORMAL_PRIORITY         1

/* ============================================================================
 * ENUMERATIONS
 * ============================================================================ */
typedef enum {
    LIGHT_RED = 0,
    LIGHT_YELLOW,
    LIGHT_GREEN,
    LIGHT_BLINKING_YELLOW,
    LIGHT_OFF
} TrafficLightState;

typedef enum {
    DIR_NORTH = 0,
    DIR_SOUTH,
    DIR_EAST,
    DIR_WEST
} Direction;

typedef enum {
    SENSOR_IR = 0,
    SENSOR_ULTRASONIC,
    SENSOR_INDUCTIVE,
    SENSOR_CAMERA,
    SENSOR_V2X,
    SENSOR_WEATHER
} SensorType;

typedef enum {
    MODE_NORMAL = 0,
    MODE_EMERGENCY,
    MODE_PEDESTRIAN,
    MODE_CONGESTION,
    MODE_NIGHT,
    MODE_FAULT
} SystemMode;

typedef enum {
    VEHICLE_CAR = 0,
    VEHICLE_BUS,
    VEHICLE_TRUCK,
    VEHICLE_EMERGENCY,
    VEHICLE_BICYCLE
} VehicleType;

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */
typedef struct {
    uint32_t id;
    SensorType type;
    Direction direction;
    uint8_t lane;
    bool active;
    float last_reading;
    uint32_t confidence;
    struct timeval last_update;
} Sensor;

typedef struct {
    uint32_t vehicle_id;
    VehicleType type;
    Direction approach;
    uint8_t lane;
    uint32_t priority;
    struct timeval arrival_time;
    bool processed;
} VehicleQueue;

typedef struct {
    Direction direction;
    TrafficLightState state;
    uint32_t timer;
    uint32_t green_duration;
    uint32_t vehicle_count;
    uint32_t pedestrian_waiting;
    bool emergency_override;
    uint32_t cycle_count;
    float avg_wait_time;
    float density_history[HISTORY_SIZE];
    uint8_t history_index;
} LaneController;

typedef struct {
    SystemMode current_mode;
    SystemMode previous_mode;
    LaneController lanes[NUM_DIRECTIONS];
    Sensor sensors[NUM_DIRECTIONS][MAX_SENSORS_PER_LANE];
    VehicleQueue queue[MAX_QUEUE_LENGTH];
    uint32_t queue_head;
    uint32_t queue_tail;
    uint32_t queue_count;
    uint32_t emergency_count;
    uint32_t total_vehicles_processed;
    uint32_t total_pedestrians_processed;
    float system_efficiency;
    bool system_fault;
    char fault_message[128];
    struct timeval start_time;
} TrafficSystem;

typedef struct {
    bool active;
    Direction target_direction;
    uint32_t vehicle_id;
    struct timeval detection_time;
} EmergencyAlert;

/* ============================================================================
 * GLOBALS (extern declarations)
 * ============================================================================ */
extern TrafficSystem g_system;
extern EmergencyAlert g_emergency_alerts[MAX_EMERGENCY_VEHICLES];
extern volatile bool g_running;

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */
const char* direction_to_str(Direction d);
const char* light_to_str(TrafficLightState s);
const char* mode_to_str(SystemMode m);
float calculate_density(LaneController *lane);
float calculate_moving_average(LaneController *lane);
void log_event(const char *format, ...);
void get_timestamp_str(char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* TRAFFIC_SYSTEM_H */
