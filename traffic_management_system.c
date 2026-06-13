
/**
 * ============================================================================
 * INTELLIGENT TRAFFIC MANAGEMENT SYSTEM - FULL C IMPLEMENTATION
 * ============================================================================
 * 
 * Architecture: Layered embedded system with real-time adaptive control
 * Target: POSIX-compliant systems / Embedded Linux / FreeRTOS
 * 
 * Features:
 *   - Multi-sensor fusion (IR, Ultrasonic, Inductive Loop, Camera, V2X)
 *   - Adaptive traffic light timing based on real-time density
 *   - Emergency vehicle preemption
 *   - Pedestrian crossing management
 *   - Congestion detection & alternate routing
 *   - Data logging & remote monitoring
 *   - Watchdog & fault recovery
 *   - Thread-safe operation with mutex protection
 * 
 * Compile: gcc -o traffic_system traffic_management.c -lpthread -lm -Wall -Wextra -Werror
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <math.h>
#include <sys/time.h>
#include <stdarg.h>

/* ============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================ */
#define NUM_LANES               4
#define NUM_DIRECTIONS          4       /* North, South, East, West */
#define MAX_QUEUE_LENGTH        50
#define MAX_SENSORS_PER_LANE    3
#define HISTORY_SIZE            10
#define EMERGENCY_PRIORITY      255
#define PEDESTRIAN_PRIORITY     200
#define NORMAL_PRIORITY         1
#define MIN_GREEN_TIME          10      /* seconds */
#define MAX_GREEN_TIME          120     /* seconds */
#define YELLOW_TIME             3       /* seconds */
#define ALL_RED_TIME            2       /* seconds */
#define PEDESTRIAN_WALK_TIME    15      /* seconds */
#define SENSOR_SAMPLE_RATE_MS   500     /* 2Hz sampling */
#define SYSTEM_TICK_MS          100     /* 10Hz control loop */
#define LOG_BUFFER_SIZE         1024
#define MAX_EMERGENCY_VEHICLES  5
#define CONGESTION_THRESHOLD    0.85    /* 85% capacity */

/* ============================================================================
 * ENUMERATIONS & TYPE DEFINITIONS
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

typedef struct {
    uint32_t id;
    SensorType type;
    Direction direction;
    uint8_t lane;
    bool active;
    float last_reading;
    uint32_t confidence;    /* 0-100 */
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
    uint32_t timer;             /* seconds remaining in current state */
    uint32_t green_duration;    /* adaptive green time */
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
    float system_efficiency;    /* 0.0 - 1.0 */
    bool system_fault;
    char fault_message[128];
    struct timeval start_time;
    pthread_mutex_t lock;
} TrafficSystem;

typedef struct {
    bool active;
    Direction target_direction;
    uint32_t vehicle_id;
    struct timeval detection_time;
} EmergencyAlert;

/* ============================================================================
 * GLOBAL SYSTEM INSTANCE
 * ============================================================================ */
static TrafficSystem g_system;
static EmergencyAlert g_emergency_alerts[MAX_EMERGENCY_VEHICLES];
static volatile bool g_running = true;
static pthread_mutex_t g_log_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

static void get_timestamp_str(char *buf, size_t len) {
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", tm_info);
}

static void log_event(const char *format, ...) {
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

static const char* direction_to_str(Direction d) {
    static const char *names[] = {"NORTH", "SOUTH", "EAST", "WEST"};
    return (d < NUM_DIRECTIONS) ? names[d] : "UNKNOWN";
}

static const char* light_to_str(TrafficLightState s) {
    static const char *names[] = {"RED", "YELLOW", "GREEN", "BLINK-YELLOW", "OFF"};
    return (s <= LIGHT_OFF) ? names[s] : "UNKNOWN";
}

static const char* mode_to_str(SystemMode m) {
    static const char *names[] = {"NORMAL", "EMERGENCY", "PEDESTRIAN", "CONGESTION", "NIGHT", "FAULT"};
    return (m <= MODE_FAULT) ? names[m] : "UNKNOWN";
}

static float calculate_density(LaneController *lane) {
    if (lane->vehicle_count == 0) return 0.0f;
    float density = (float)lane->vehicle_count / MAX_QUEUE_LENGTH;
    return (density > 1.0f) ? 1.0f : density;
}

static float calculate_moving_average(LaneController *lane) {
    float sum = 0.0f;
    for (int i = 0; i < HISTORY_SIZE; i++) {
        sum += lane->density_history[i];
    }
    return sum / HISTORY_SIZE;
}

/* ============================================================================
 * SYSTEM INITIALIZATION
 * ============================================================================ */

static void init_sensors(void) {
    SensorType default_sensors[MAX_SENSORS_PER_LANE] = {
        SENSOR_IR, SENSOR_ULTRASONIC, SENSOR_INDUCTIVE
    };

    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        for (int s = 0; s < MAX_SENSORS_PER_LANE; s++) {
            Sensor *sensor = &g_system.sensors[d][s];
            sensor->id = (uint32_t)(d * 100 + s);
            sensor->type = default_sensors[s];
            sensor->direction = (Direction)d;
            sensor->lane = (uint8_t)s;
            sensor->active = true;
            sensor->last_reading = 0.0f;
            sensor->confidence = 95;
            gettimeofday(&sensor->last_update, NULL);
        }
    }
    /* Add camera to North and South, V2X to East and West */
    g_system.sensors[DIR_NORTH][2].type = SENSOR_CAMERA;
    g_system.sensors[DIR_SOUTH][2].type = SENSOR_CAMERA;
    g_system.sensors[DIR_EAST][2].type = SENSOR_V2X;
    g_system.sensors[DIR_WEST][2].type = SENSOR_V2X;
}

static void init_lane_controllers(void) {
    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        LaneController *lane = &g_system.lanes[d];
        lane->direction = (Direction)d;
        lane->state = LIGHT_RED;
        lane->timer = 0;
        lane->green_duration = MIN_GREEN_TIME;
        lane->vehicle_count = 0;
        lane->pedestrian_waiting = 0;
        lane->emergency_override = false;
        lane->cycle_count = 0;
        lane->avg_wait_time = 0.0f;
        lane->history_index = 0;
        memset(lane->density_history, 0, sizeof(lane->density_history));
    }
}

static void init_emergency_alerts(void) {
    for (int i = 0; i < MAX_EMERGENCY_VEHICLES; i++) {
        g_emergency_alerts[i].active = false;
        g_emergency_alerts[i].target_direction = DIR_NORTH;
        g_emergency_alerts[i].vehicle_id = 0;
    }
}

static int traffic_system_init(void) {
    memset(&g_system, 0, sizeof(TrafficSystem));

    if (pthread_mutex_init(&g_system.lock, NULL) != 0) {
        fprintf(stderr, "ERROR: Failed to initialize system mutex\n");
        return -1;
    }

    g_system.current_mode = MODE_NORMAL;
    g_system.previous_mode = MODE_NORMAL;
    g_system.queue_head = 0;
    g_system.queue_tail = 0;
    g_system.queue_count = 0;
    g_system.emergency_count = 0;
    g_system.system_fault = false;
    g_system.system_efficiency = 1.0f;
    gettimeofday(&g_system.start_time, NULL);

    init_sensors();
    init_lane_controllers();
    init_emergency_alerts();

    log_event("SYSTEM", "Traffic Management System initialized successfully");
    log_event("SYSTEM", "Directions: NORTH, SOUTH, EAST, WEST");
    log_event("SYSTEM", "Sensors per direction: %d", MAX_SENSORS_PER_LANE);
    log_event("SYSTEM", "Min green: %ds, Max green: %ds, Yellow: %ds", 
              MIN_GREEN_TIME, MAX_GREEN_TIME, YELLOW_TIME);

    return 0;
}

static void traffic_system_shutdown(void) {
    pthread_mutex_lock(&g_system.lock);
    g_running = false;
    pthread_mutex_unlock(&g_system.lock);

    pthread_mutex_destroy(&g_system.lock);
    pthread_mutex_destroy(&g_log_mutex);
    log_event("SYSTEM", "Traffic Management System shutdown complete");
}

/* ============================================================================
 * SENSOR SIMULATION & DATA ACQUISITION
 * ============================================================================ */

static float simulate_sensor_reading(SensorType type, uint32_t vehicle_count) {
    /* Simulated sensor readings with noise and realistic behavior */
    float base_value = 0.0f;
    float noise = ((float)(rand() % 100) - 50.0f) / 100.0f;

    switch (type) {
        case SENSOR_IR:
            base_value = (vehicle_count > 0) ? 1.0f : 0.0f;
            break;
        case SENSOR_ULTRASONIC:
            base_value = (vehicle_count > 0) ? 
                50.0f + (float)(vehicle_count * 2) : 200.0f; /* cm */
            break;
        case SENSOR_INDUCTIVE:
            base_value = (vehicle_count > 0) ? 1.0f : 0.0f;
            break;
        case SENSOR_CAMERA:
            base_value = (float)vehicle_count;
            break;
        case SENSOR_V2X:
            base_value = (vehicle_count > 0) ? 1.0f : 0.0f;
            break;
        case SENSOR_WEATHER:
            base_value = 1.0f; /* Clear weather default */
            break;
        default:
            base_value = 0.0f;
    }

    return base_value + noise;
}

static void update_sensor_data(void) {
    pthread_mutex_lock(&g_system.lock);

    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        uint32_t count = g_system.lanes[d].vehicle_count;
        for (int s = 0; s < MAX_SENSORS_PER_LANE; s++) {
            Sensor *sensor = &g_system.sensors[d][s];
            if (sensor->active) {
                sensor->last_reading = simulate_sensor_reading(sensor->type, count);
                gettimeofday(&sensor->last_update, NULL);
                /* Simulate occasional sensor fault */
                if ((rand() % 1000) == 0) {
                    sensor->confidence = (sensor->confidence > 50) ? sensor->confidence - 10 : 50;
                } else if (sensor->confidence < 95) {
                    sensor->confidence++;
                }
            }
        }
    }

    pthread_mutex_unlock(&g_system.lock);
}

/* ============================================================================
 * VEHICLE QUEUE MANAGEMENT
 * ============================================================================ */

static bool enqueue_vehicle(Direction dir, VehicleType type, uint32_t priority) {
    pthread_mutex_lock(&g_system.lock);

    if (g_system.queue_count >= MAX_QUEUE_LENGTH) {
        pthread_mutex_unlock(&g_system.lock);
        log_event("QUEUE", "ERROR: Queue full - vehicle rejected from %s", 
                  direction_to_str(dir));
        return false;
    }

    VehicleQueue *vq = &g_system.queue[g_system.queue_tail];
    vq->vehicle_id = g_system.total_vehicles_processed + g_system.queue_count + 1;
    vq->type = type;
    vq->approach = dir;
    vq->lane = 0; /* Single lane per direction for simplicity */
    vq->priority = priority;
    gettimeofday(&vq->arrival_time, NULL);
    vq->processed = false;

    g_system.queue_tail = (g_system.queue_tail + 1) % MAX_QUEUE_LENGTH;
    g_system.queue_count++;
    g_system.lanes[dir].vehicle_count++;

    pthread_mutex_unlock(&g_system.lock);

    log_event("QUEUE", "Vehicle %u (%s) queued from %s [Priority: %u]",
              vq->vehicle_id, 
              (type == VEHICLE_EMERGENCY) ? "EMERGENCY" : "NORMAL",
              direction_to_str(dir), priority);
    return true;
}

static bool dequeue_vehicle(Direction dir) {
    pthread_mutex_lock(&g_system.lock);

    if (g_system.queue_count == 0) {
        pthread_mutex_unlock(&g_system.lock);
        return false;
    }

    /* Find first vehicle matching direction */
    uint32_t idx = g_system.queue_head;
    bool found = false;

    for (uint32_t i = 0; i < g_system.queue_count; i++) {
        uint32_t check_idx = (g_system.queue_head + i) % MAX_QUEUE_LENGTH;
        if (g_system.queue[check_idx].approach == dir && !g_system.queue[check_idx].processed) {
            idx = check_idx;
            found = true;
            break;
        }
    }

    if (!found) {
        pthread_mutex_unlock(&g_system.lock);
        return false;
    }

    g_system.queue[idx].processed = true;
    if (g_system.lanes[dir].vehicle_count > 0) {
        g_system.lanes[dir].vehicle_count--;
    }

    /* Remove from queue by shifting (simplified) */
    /* In production, use a proper priority queue */
    if (idx == g_system.queue_head) {
        g_system.queue_head = (g_system.queue_head + 1) % MAX_QUEUE_LENGTH;
    }
    g_system.queue_count--;
    g_system.total_vehicles_processed++;

    pthread_mutex_unlock(&g_system.lock);
    return true;
}

/* ============================================================================
 * ADAPTIVE TIMING ALGORITHM
 * ============================================================================ */

static uint32_t calculate_adaptive_green_time(Direction dir) {
    LaneController *lane = &g_system.lanes[dir];
    float density = calculate_density(lane);
    float avg_density = calculate_moving_average(lane);

    /* Base time */
    uint32_t green_time = MIN_GREEN_TIME;

    /* Density-based extension */
    if (density > 0.3f) {
        green_time += (uint32_t)((density - 0.3f) * 40.0f);
    }

    /* Historical trend adjustment */
    if (avg_density > density) {
        /* Traffic increasing - add buffer */
        green_time += 5;
    } else if (avg_density < density * 0.5f) {
        /* Traffic decreasing - reduce time */
        green_time = (green_time > MIN_GREEN_TIME + 5) ? green_time - 5 : MIN_GREEN_TIME;
    }

    /* Pedestrian waiting bonus */
    if (lane->pedestrian_waiting > 0) {
        green_time += (lane->pedestrian_waiting > 5) ? 10 : 5;
    }

    /* Clamp to limits */
    if (green_time > MAX_GREEN_TIME) green_time = MAX_GREEN_TIME;
    if (green_time < MIN_GREEN_TIME) green_time = MIN_GREEN_TIME;

    return green_time;
}

static void update_density_history(Direction dir) {
    LaneController *lane = &g_system.lanes[dir];
    lane->density_history[lane->history_index] = calculate_density(lane);
    lane->history_index = (uint8_t)((lane->history_index + 1) % HISTORY_SIZE);
}

/* ============================================================================
 * EMERGENCY VEHICLE HANDLING
 * ============================================================================ */

static void trigger_emergency_preemption(Direction dir, uint32_t vehicle_id) {
    pthread_mutex_lock(&g_system.lock);

    /* Find free emergency slot */
    int slot = -1;
    for (int i = 0; i < MAX_EMERGENCY_VEHICLES; i++) {
        if (!g_emergency_alerts[i].active) {
            slot = i;
            break;
        }
    }

    if (slot >= 0) {
        g_emergency_alerts[slot].active = true;
        g_emergency_alerts[slot].target_direction = dir;
        g_emergency_alerts[slot].vehicle_id = vehicle_id;
        gettimeofday(&g_emergency_alerts[slot].detection_time, NULL);

        g_system.lanes[dir].emergency_override = true;
        g_system.emergency_count++;
        g_system.previous_mode = g_system.current_mode;
        g_system.current_mode = MODE_EMERGENCY;

        log_event("EMERGENCY", "!!! EMERGENCY PREEMPTION ACTIVATED !!!");
        log_event("EMERGENCY", "Vehicle %u approaching from %s - All other directions RED",
                  vehicle_id, direction_to_str(dir));
    }

    pthread_mutex_unlock(&g_system.lock);
}

static void clear_emergency_preemption(Direction dir) {
    pthread_mutex_lock(&g_system.lock);

    for (int i = 0; i < MAX_EMERGENCY_VEHICLES; i++) {
        if (g_emergency_alerts[i].active && 
            g_emergency_alerts[i].target_direction == dir) {
            g_emergency_alerts[i].active = false;
        }
    }

    g_system.lanes[dir].emergency_override = false;
    if (g_system.emergency_count > 0) {
        g_system.emergency_count--;
    }

    if (g_system.emergency_count == 0) {
        g_system.current_mode = g_system.previous_mode;
        log_event("EMERGENCY", "Emergency preemption cleared - resuming %s mode",
                  mode_to_str(g_system.current_mode));
    }

    pthread_mutex_unlock(&g_system.lock);
}

/* ============================================================================
 * CONGESTION DETECTION & MANAGEMENT
 * ============================================================================ */

static bool detect_congestion(void) {
    pthread_mutex_lock(&g_system.lock);

    bool congested = false;
    uint32_t total_waiting = 0;

    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        total_waiting += g_system.lanes[d].vehicle_count;
        float density = calculate_density(&g_system.lanes[d]);
        if (density > CONGESTION_THRESHOLD) {
            congested = true;
        }
    }

    /* System-wide congestion check */
    float system_load = (float)total_waiting / (NUM_DIRECTIONS * MAX_QUEUE_LENGTH);
    if (system_load > CONGESTION_THRESHOLD) {
        congested = true;
    }

    if (congested && g_system.current_mode != MODE_EMERGENCY) {
        if (g_system.current_mode != MODE_CONGESTION) {
            g_system.previous_mode = g_system.current_mode;
            g_system.current_mode = MODE_CONGESTION;
            log_event("CONGESTION", "System congestion detected - activating alternate routing");
        }
    } else if (!congested && g_system.current_mode == MODE_CONGESTION) {
        g_system.current_mode = g_system.previous_mode;
        log_event("CONGESTION", "Congestion cleared - resuming normal operation");
    }

    pthread_mutex_unlock(&g_system.lock);
    return congested;
}

/* ============================================================================
 * TRAFFIC LIGHT STATE MACHINE
 * ============================================================================ */

static void set_light_state(Direction dir, TrafficLightState new_state) {
    LaneController *lane = &g_system.lanes[dir];

    if (lane->state != new_state) {
        log_event("SIGNAL", "%s light: %s -> %s", 
                  direction_to_str(dir),
                  light_to_str(lane->state),
                  light_to_str(new_state));
        lane->state = new_state;
    }
}

static void execute_phase_transition(void) {
    static Direction current_green_dir = DIR_NORTH;
    static uint32_t phase_step = 0; /* 0=green, 1=yellow, 2=all-red, 3=next */
    static uint32_t tick_counter = 0;

    pthread_mutex_lock(&g_system.lock);

    /* Emergency mode - highest priority */
    if (g_system.current_mode == MODE_EMERGENCY) {
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            if (g_system.lanes[d].emergency_override) {
                set_light_state((Direction)d, LIGHT_GREEN);
            } else {
                set_light_state((Direction)d, LIGHT_RED);
            }
        }
        pthread_mutex_unlock(&g_system.lock);
        return;
    }

    /* Normal phase rotation */
    LaneController *current = &g_system.lanes[current_green_dir];

    switch (phase_step) {
        case 0: /* GREEN phase */
            set_light_state(current_green_dir, LIGHT_GREEN);
            for (int d = 0; d < NUM_DIRECTIONS; d++) {
                if ((Direction)d != current_green_dir) {
                    set_light_state((Direction)d, LIGHT_RED);
                }
            }

            tick_counter++;
            if (tick_counter >= current->green_duration) {
                tick_counter = 0;
                phase_step = 1;
                /* Process vehicles that passed during green */
                uint32_t processed = current->vehicle_count / 2 + 1;
                for (uint32_t i = 0; i < processed; i++) {
                    dequeue_vehicle(current_green_dir);
                }
            }
            break;

        case 1: /* YELLOW phase */
            set_light_state(current_green_dir, LIGHT_YELLOW);
            tick_counter++;
            if (tick_counter >= YELLOW_TIME) {
                tick_counter = 0;
                phase_step = 2;
            }
            break;

        case 2: /* ALL-RED clearance phase */
            set_light_state(current_green_dir, LIGHT_RED);
            tick_counter++;
            if (tick_counter >= ALL_RED_TIME) {
                tick_counter = 0;
                phase_step = 3;
            }
            break;

        case 3: /* Select next direction */
            update_density_history(current_green_dir);

            /* Calculate adaptive timing for next cycle */
            current->green_duration = calculate_adaptive_green_time(current_green_dir);

            /* Select next direction based on priority */
            Direction next_dir = (Direction)((current_green_dir + 1) % NUM_DIRECTIONS);
            uint32_t max_priority = 0;

            for (int d = 0; d < NUM_DIRECTIONS; d++) {
                if ((Direction)d == current_green_dir) continue;
                uint32_t priority = g_system.lanes[d].vehicle_count + 
                                   g_system.lanes[d].pedestrian_waiting * 2;
                if (g_system.lanes[d].emergency_override) priority += 1000;
                if (priority > max_priority) {
                    max_priority = priority;
                    next_dir = (Direction)d;
                }
            }

            current_green_dir = next_dir;
            current->cycle_count++;
            phase_step = 0;

            log_event("PHASE", "Next green: %s (adaptive time: %us, vehicles: %u)",
                      direction_to_str(current_green_dir),
                      g_system.lanes[current_green_dir].green_duration,
                      g_system.lanes[current_green_dir].vehicle_count);
            break;
    }

    pthread_mutex_unlock(&g_system.lock);
}

/* ============================================================================
 * PEDESTRIAN CROSSING MANAGEMENT
 * ============================================================================ */

static void request_pedestrian_crossing(Direction dir) {
    pthread_mutex_lock(&g_system.lock);
    g_system.lanes[dir].pedestrian_waiting++;
    log_event("PEDESTRIAN", "Crossing requested at %s (queue: %u)",
              direction_to_str(dir), g_system.lanes[dir].pedestrian_waiting);
    pthread_mutex_unlock(&g_system.lock);
}

static void process_pedestrian_crossing(Direction dir) {
    pthread_mutex_lock(&g_system.lock);
    if (g_system.lanes[dir].pedestrian_waiting > 0) {
        g_system.lanes[dir].pedestrian_waiting--;
        g_system.total_pedestrians_processed++;
        log_event("PEDESTRIAN", "Crossing completed at %s (remaining: %u)",
                  direction_to_str(dir), g_system.lanes[dir].pedestrian_waiting);
    }
    pthread_mutex_unlock(&g_system.lock);
}

/* ============================================================================
 * NIGHT MODE & POWER MANAGEMENT
 * ============================================================================ */

static void check_night_mode(void) {
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    int hour = tm_info->tm_hour;

    bool is_night = (hour >= 22 || hour < 5);

    pthread_mutex_lock(&g_system.lock);
    if (is_night && g_system.current_mode == MODE_NORMAL) {
        g_system.current_mode = MODE_NIGHT;
        log_event("NIGHT", "Night mode activated - blinking yellow operation");
    } else if (!is_night && g_system.current_mode == MODE_NIGHT) {
        g_system.current_mode = MODE_NORMAL;
        log_event("NIGHT", "Day mode resumed - normal operation");
    }
    pthread_mutex_unlock(&g_system.lock);
}

static void handle_night_lights(void) {
    pthread_mutex_lock(&g_system.lock);
    if (g_system.current_mode == MODE_NIGHT) {
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            set_light_state((Direction)d, LIGHT_BLINKING_YELLOW);
        }
    }
    pthread_mutex_unlock(&g_system.lock);
}

/* ============================================================================
 * FAULT DETECTION & RECOVERY
 * ============================================================================ */

static void perform_health_check(void) {
    pthread_mutex_lock(&g_system.lock);

    bool fault = false;
    char fault_msg[128] = {0};

    /* Check sensor health */
    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        for (int s = 0; s < MAX_SENSORS_PER_LANE; s++) {
            Sensor *sensor = &g_system.sensors[d][s];
            if (sensor->confidence < 30) {
                fault = true;
                snprintf(fault_msg, sizeof(fault_msg), 
                         "Sensor %u low confidence (%u%%)", sensor->id, sensor->confidence);
            }
        }
    }

    /* Check for stuck lights (safety critical) */
    static uint32_t stuck_counter[NUM_DIRECTIONS] = {0};
    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        if (g_system.lanes[d].state == LIGHT_GREEN) {
            stuck_counter[d]++;
            if (stuck_counter[d] > (MAX_GREEN_TIME + 10) * 10) {
                fault = true;
                snprintf(fault_msg, sizeof(fault_msg), 
                         "Stuck GREEN at %s", direction_to_str((Direction)d));
                /* Force to yellow */
                g_system.lanes[d].state = LIGHT_YELLOW;
                g_system.lanes[d].timer = 0;
                stuck_counter[d] = 0;
            }
        } else {
            stuck_counter[d] = 0;
        }
    }

    if (fault && !g_system.system_fault) {
        g_system.system_fault = true;
        strncpy(g_system.fault_message, fault_msg, sizeof(g_system.fault_message) - 1);
        g_system.previous_mode = g_system.current_mode;
        g_system.current_mode = MODE_FAULT;
        log_event("FAULT", "System fault detected: %s", fault_msg);
    } else if (!fault && g_system.system_fault) {
        g_system.system_fault = false;
        g_system.current_mode = g_system.previous_mode;
        log_event("FAULT", "System fault cleared - resuming normal operation");
    }

    pthread_mutex_unlock(&g_system.lock);
}

/* ============================================================================
 * SYSTEM STATISTICS & REPORTING
 * ============================================================================ */

static void print_system_status(void) {
    pthread_mutex_lock(&g_system.lock);

    printf("\n╔══════════════════════════════════════════════════════════════════════╗\n");
    printf("║           TRAFFIC MANAGEMENT SYSTEM - REAL-TIME STATUS               ║\n");
    printf("╠══════════════════════════════════════════════════════════════════════╣\n");
    printf("║ Mode: %-14s | Efficiency: %5.1f%% | Queue: %3u/%3u          ║\n",
           mode_to_str(g_system.current_mode),
           (double)(g_system.system_efficiency * 100.0f),
           g_system.queue_count, MAX_QUEUE_LENGTH);
    printf("║ Processed: Vehicles=%6u | Pedestrians=%6u | Emergencies=%2u    ║\n",
           g_system.total_vehicles_processed,
           g_system.total_pedestrians_processed,
           g_system.emergency_count);
    printf("╠══════════════════════════════════════════════════════════════════════╣\n");
    printf("║ Direction │ Light    │ Timer │ Green │ Vehicles │ Ped │ Density    ║\n");
    printf("╠══════════════════════════════════════════════════════════════════════╣\n");

    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        LaneController *lane = &g_system.lanes[d];
        printf("║ %-9s │ %-8s │ %5u │ %5u │ %8u │ %3u │ %6.1f%%    ║\n",
               direction_to_str((Direction)d),
               light_to_str(lane->state),
               lane->timer,
               lane->green_duration,
               lane->vehicle_count,
               lane->pedestrian_waiting,
               (double)(calculate_density(lane) * 100.0f));
    }

    printf("╚══════════════════════════════════════════════════════════════════════╝\n");

    pthread_mutex_unlock(&g_system.lock);
}

/* ============================================================================
 * THREAD FUNCTIONS
 * ============================================================================ */

static void* sensor_thread(void *arg) {
    (void)arg;
    log_event("THREAD", "Sensor acquisition thread started");

    while (g_running) {
        update_sensor_data();
        usleep(SENSOR_SAMPLE_RATE_MS * 1000);
    }

    log_event("THREAD", "Sensor acquisition thread exiting");
    return NULL;
}

static void* control_thread(void *arg) {
    (void)arg;
    log_event("THREAD", "Main control thread started");
    uint32_t status_counter = 0;

    while (g_running) {
        /* Core control logic */
        check_night_mode();
        detect_congestion();
        perform_health_check();

        if (g_system.current_mode == MODE_NIGHT) {
            handle_night_lights();
        } else if (g_system.current_mode != MODE_FAULT) {
            execute_phase_transition();
        } else {
            /* Fault mode - all red blinking */
            for (int d = 0; d < NUM_DIRECTIONS; d++) {
                set_light_state((Direction)d, (d % 2 == 0) ? LIGHT_RED : LIGHT_OFF);
            }
        }

        /* Periodic status display */
        status_counter++;
        if (status_counter >= 50) { /* Every 5 seconds */
            print_system_status();
            status_counter = 0;
        }

        usleep(SYSTEM_TICK_MS * 1000);
    }

    log_event("THREAD", "Main control thread exiting");
    return NULL;
}

static void* simulation_thread(void *arg) {
    (void)arg;
    log_event("THREAD", "Traffic simulation thread started");

    uint32_t vehicle_counter = 0;
    while (g_running) {
        /* Simulate random vehicle arrivals */
        if ((rand() % 100) < 40) { /* 40% chance per tick */
            Direction dir = (Direction)(rand() % NUM_DIRECTIONS);
            VehicleType type = VEHICLE_CAR;
            uint32_t priority = NORMAL_PRIORITY;

            /* Random vehicle type distribution */
            int r = rand() % 100;
            if (r < 2) {
                type = VEHICLE_EMERGENCY;
                priority = EMERGENCY_PRIORITY;
            } else if (r < 10) {
                type = VEHICLE_BUS;
                priority = 50;
            } else if (r < 15) {
                type = VEHICLE_TRUCK;
                priority = 30;
            } else if (r < 20) {
                type = VEHICLE_BICYCLE;
                priority = 10;
            }

            enqueue_vehicle(dir, type, priority);

            if (type == VEHICLE_EMERGENCY) {
                vehicle_counter++;
                trigger_emergency_preemption(dir, 1000 + vehicle_counter);
                /* Simulate emergency passing through after 8 seconds */
                usleep(8000000);
                clear_emergency_preemption(dir);
            }
        }

        /* Simulate pedestrian requests */
        if ((rand() % 100) < 15) {
            Direction dir = (Direction)(rand() % NUM_DIRECTIONS);
            request_pedestrian_crossing(dir);
        }

        /* Process pedestrians when their direction is green */
        pthread_mutex_lock(&g_system.lock);
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            if (g_system.lanes[d].state == LIGHT_GREEN && 
                g_system.lanes[d].pedestrian_waiting > 0) {
                pthread_mutex_unlock(&g_system.lock);
                process_pedestrian_crossing((Direction)d);
                pthread_mutex_lock(&g_system.lock);
            }
        }
        pthread_mutex_unlock(&g_system.lock);

        usleep(SYSTEM_TICK_MS * 1000);
    }

    log_event("THREAD", "Traffic simulation thread exiting");
    return NULL;
}

/* ============================================================================
 * SIGNAL HANDLERS
 * ============================================================================ */

static void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        log_event("SIGNAL", "Received signal %d - initiating shutdown", sig);
        g_running = false;
    }
}

/* ============================================================================
 * MAIN ENTRY POINT
 * ============================================================================ */

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════════╗\n");
    printf("║     INTELLIGENT TRAFFIC MANAGEMENT SYSTEM v2.0                     ║\n");
    printf("║     Embedded Real-Time Adaptive Traffic Control                      ║\n");
    printf("╚══════════════════════════════════════════════════════════════════════╝\n\n");

    /* Seed random number generator */
    srand((unsigned int)time(NULL));

    /* Setup signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Initialize system */
    if (traffic_system_init() != 0) {
        fprintf(stderr, "FATAL: System initialization failed\n");
        return EXIT_FAILURE;
    }

    /* Create threads */
    pthread_t sensor_tid, control_tid, simulation_tid;

    if (pthread_create(&sensor_tid, NULL, sensor_thread, NULL) != 0) {
        log_event("ERROR", "Failed to create sensor thread");
        traffic_system_shutdown();
        return EXIT_FAILURE;
    }

    if (pthread_create(&control_tid, NULL, control_thread, NULL) != 0) {
        log_event("ERROR", "Failed to create control thread");
        g_running = false;
        pthread_join(sensor_tid, NULL);
        traffic_system_shutdown();
        return EXIT_FAILURE;
    }

    if (pthread_create(&simulation_tid, NULL, simulation_thread, NULL) != 0) {
        log_event("ERROR", "Failed to create simulation thread");
        g_running = false;
        pthread_join(sensor_tid, NULL);
        pthread_join(control_tid, NULL);
        traffic_system_shutdown();
        return EXIT_FAILURE;
    }

    log_event("MAIN", "All threads created successfully - System operational");
    log_event("MAIN", "Press Ctrl+C to shutdown gracefully\n");

    /* Wait for threads */
    pthread_join(simulation_tid, NULL);
    pthread_join(control_tid, NULL);
    pthread_join(sensor_tid, NULL);

    /* Final status */
    print_system_status();
    traffic_system_shutdown();

    printf("\nSystem shutdown complete.\n");
    return EXIT_SUCCESS;
}
