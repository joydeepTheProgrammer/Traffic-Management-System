/**
 * @file main.c
 * @brief POSIX simulation entry point for the traffic management system.
 */

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "traffic_system.h"
#include "hal.h"
#include "protocol.h"
#include "watchdog.h"

#define TASK_SENSOR       1U
#define TASK_CONTROL      2U
#define TASK_COMM         3U
#define TASK_DIAGNOSTICS  4U
#define TASK_SIMULATION   5U
#define CONTROL_TICK_MS   100U
#define GREEN_TICKS(seconds) ((seconds) * (1000U / CONTROL_TICK_MS))

static pthread_mutex_t system_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile sig_atomic_t system_running = 1;

static uint32_t light_base(Direction direction) {
    return TRAFFIC_NORTH_RED + ((uint32_t)direction * 3U);
}

static void set_light_locked(Direction direction, TrafficLightState state) {
    uint32_t base = light_base(direction);
    hal_gpio_write(base, state == LIGHT_RED);
    hal_gpio_write(base + 1U, state == LIGHT_YELLOW || state == LIGHT_BLINKING_YELLOW);
    hal_gpio_write(base + 2U, state == LIGHT_GREEN);
    g_system.lanes[direction].state = state;
}

static void set_all_red_locked(void) {
    for (Direction direction = DIR_NORTH; direction < NUM_DIRECTIONS; direction++) {
        set_light_locked(direction, LIGHT_RED);
    }
}

static bool enqueue_vehicle_locked(Direction direction, VehicleType type, uint32_t priority) {
    if (direction >= NUM_DIRECTIONS || g_system.queue_count >= MAX_QUEUE_LENGTH) return false;

    VehicleQueue *vehicle = &g_system.queue[g_system.queue_tail];
    memset(vehicle, 0, sizeof(*vehicle));
    vehicle->vehicle_id = g_system.total_vehicles_processed + g_system.queue_count + 1U;
    vehicle->type = type;
    vehicle->approach = direction;
    vehicle->priority = priority;
    gettimeofday(&vehicle->arrival_time, NULL);
    g_system.queue_tail = (g_system.queue_tail + 1U) % MAX_QUEUE_LENGTH;
    g_system.queue_count++;
    g_system.lanes[direction].vehicle_count++;
    return true;
}

/* Remove the first queued vehicle for a direction without leaving ring-buffer holes. */
static bool dequeue_vehicle_locked(Direction direction) {
    if (direction >= NUM_DIRECTIONS || g_system.queue_count == 0U) return false;

    uint32_t offset;
    for (offset = 0; offset < g_system.queue_count; offset++) {
        uint32_t index = (g_system.queue_head + offset) % MAX_QUEUE_LENGTH;
        if (g_system.queue[index].approach == direction) break;
    }
    if (offset == g_system.queue_count) return false;

    for (uint32_t i = offset; i + 1U < g_system.queue_count; i++) {
        uint32_t destination = (g_system.queue_head + i) % MAX_QUEUE_LENGTH;
        uint32_t source = (g_system.queue_head + i + 1U) % MAX_QUEUE_LENGTH;
        g_system.queue[destination] = g_system.queue[source];
    }
    g_system.queue_tail = (g_system.queue_tail + MAX_QUEUE_LENGTH - 1U) % MAX_QUEUE_LENGTH;
    memset(&g_system.queue[g_system.queue_tail], 0, sizeof(g_system.queue[0]));
    g_system.queue_count--;
    if (g_system.lanes[direction].vehicle_count > 0U) g_system.lanes[direction].vehicle_count--;
    g_system.total_vehicles_processed++;
    return true;
}

static void activate_emergency_locked(Direction direction) {
    if (direction >= NUM_DIRECTIONS || g_system.lanes[direction].emergency_override) return;
    if (g_system.emergency_count == 0U) g_system.previous_mode = g_system.current_mode;
    g_system.lanes[direction].emergency_override = true;
    g_system.emergency_count++;
    g_system.current_mode = MODE_EMERGENCY;
}

static void clear_emergency_locked(Direction direction) {
    if (direction >= NUM_DIRECTIONS || !g_system.lanes[direction].emergency_override) return;
    g_system.lanes[direction].emergency_override = false;
    g_system.emergency_count--;
    if (g_system.emergency_count == 0U) g_system.current_mode = g_system.previous_mode;
}

static void update_mode_locked(void) {
    time_t now = time(NULL);
    struct tm *local_time = localtime(&now);
    if (local_time == NULL) return;
    bool night = local_time->tm_hour >= 22 || local_time->tm_hour < 5;
    uint32_t waiting = 0U;
    for (Direction direction = DIR_NORTH; direction < NUM_DIRECTIONS; direction++) {
        waiting += g_system.lanes[direction].vehicle_count;
    }
    float load = (float)waiting / (float)(NUM_DIRECTIONS * MAX_QUEUE_LENGTH);

    if (g_system.current_mode != MODE_EMERGENCY) {
        if (night) {
            if (g_system.current_mode != MODE_NIGHT) g_system.previous_mode = g_system.current_mode;
            g_system.current_mode = MODE_NIGHT;
        } else if (g_system.current_mode == MODE_NIGHT) {
            g_system.current_mode = MODE_NORMAL;
        } else if (load >= (CONGESTION_THRESHOLD / 100.0f)) {
            if (g_system.current_mode != MODE_CONGESTION) g_system.previous_mode = g_system.current_mode;
            g_system.current_mode = MODE_CONGESTION;
        } else if (g_system.current_mode == MODE_CONGESTION) {
            g_system.current_mode = g_system.previous_mode == MODE_CONGESTION ? MODE_NORMAL : g_system.previous_mode;
        }
    }
    g_system.system_efficiency = 1.0f - load;
}

static uint32_t adaptive_green_locked(Direction direction) {
    LaneController *lane = &g_system.lanes[direction];
    float density = calculate_density(lane);
    lane->density_history[lane->history_index] = density;
    lane->history_index = (uint8_t)((lane->history_index + 1U) % HISTORY_SIZE);
    uint32_t green = MIN_GREEN_TIME + (uint32_t)(density * 30.0f);
    if (lane->pedestrian_waiting > 0U) green += lane->pedestrian_waiting > 5U ? 10U : 5U;
    return green > MAX_GREEN_TIME ? MAX_GREEN_TIME : green;
}

static void *sensor_task(void *unused) {
    (void)unused;
    log_event("SENSOR", "Sensor task started");
    while (system_running) {
        wdg_pet(TASK_SENSOR);
        pthread_mutex_lock(&system_lock);
        for (Direction direction = DIR_NORTH; direction < NUM_DIRECTIONS; direction++) {
            for (uint32_t sensor_index = 0; sensor_index < MAX_SENSORS_PER_LANE; sensor_index++) {
                Sensor *sensor = &g_system.sensors[direction][sensor_index];
                if (!sensor->active) continue;
                if (sensor->type == SENSOR_IR) sensor->last_reading = hal_gpio_read(SENSOR_IR_NORTH + direction) ? 1.0f : 0.0f;
                else if (sensor->type == SENSOR_INDUCTIVE) sensor->last_reading = hal_adc_read_voltage(ADC_CHANNEL_0 + direction);
                else sensor->last_reading = (float)g_system.lanes[direction].vehicle_count;
                gettimeofday(&sensor->last_update, NULL);
            }
        }
        pthread_mutex_unlock(&system_lock);
        hal_delay_ms(SENSOR_SAMPLE_RATE_MS);
    }
    return NULL;
}

static void *control_task(void *unused) {
    (void)unused;
    Direction current = DIR_NORTH;
    uint32_t phase = 0U;
    uint32_t ticks = 0U;
    bool blink = false;
    log_event("CONTROL", "Control task started");
    while (system_running) {
        wdg_pet(TASK_CONTROL);
        pthread_mutex_lock(&system_lock);
        update_mode_locked();
        if (g_system.current_mode == MODE_EMERGENCY) {
            for (Direction d = DIR_NORTH; d < NUM_DIRECTIONS; d++) {
                set_light_locked(d, g_system.lanes[d].emergency_override ? LIGHT_GREEN : LIGHT_RED);
            }
        } else if (g_system.current_mode == MODE_NIGHT) {
            blink = !blink;
            for (Direction d = DIR_NORTH; d < NUM_DIRECTIONS; d++) {
                set_light_locked(d, blink ? LIGHT_BLINKING_YELLOW : LIGHT_OFF);
            }
        } else {
            LaneController *lane = &g_system.lanes[current];
            if (phase == 0U) {
                set_all_red_locked();
                set_light_locked(current, LIGHT_GREEN);
                if (++ticks >= GREEN_TICKS(lane->green_duration)) {
                    ticks = 0U;
                    phase = 1U;
                    while (dequeue_vehicle_locked(current)) { }
                }
            } else if (phase == 1U) {
                set_light_locked(current, LIGHT_YELLOW);
                if (++ticks >= GREEN_TICKS(YELLOW_TIME)) { ticks = 0U; phase = 2U; }
            } else if (phase == 2U) {
                set_all_red_locked();
                if (++ticks >= GREEN_TICKS(ALL_RED_TIME)) { ticks = 0U; phase = 3U; }
            } else {
                Direction next = (Direction)((current + 1U) % NUM_DIRECTIONS);
                uint32_t best = 0U;
                for (Direction d = DIR_NORTH; d < NUM_DIRECTIONS; d++) {
                    if (d == current) continue;
                    uint32_t score = g_system.lanes[d].vehicle_count + 2U * g_system.lanes[d].pedestrian_waiting;
                    if (score > best) { best = score; next = d; }
                }
                current = next;
                g_system.lanes[current].green_duration = adaptive_green_locked(current);
                phase = 0U;
            }
        }
        pthread_mutex_unlock(&system_lock);
        wdg_check_all();
        hal_delay_ms(CONTROL_TICK_MS);
    }
    return NULL;
}

static void process_frame(uint8_t uart_id, const ProtocolFrame *frame) {
    pthread_mutex_lock(&system_lock);
    switch (frame->cmd) {
        case CMD_HEARTBEAT:
            protocol_send_response(uart_id, CMD_HEARTBEAT, RESP_OK, NULL, 0U);
            break;
        case CMD_STATUS_REQ:
            protocol_send_status(uart_id);
            break;
        case CMD_SET_LIGHT:
            if (frame->length < sizeof(LightControlPayload)) protocol_send_error(uart_id, RESP_INVALID_PARAM);
            else {
                const LightControlPayload *payload = (const LightControlPayload *)frame->payload;
                if (payload->direction >= NUM_DIRECTIONS || payload->state > LIGHT_GREEN) protocol_send_error(uart_id, RESP_INVALID_PARAM);
                else { set_light_locked((Direction)payload->direction, (TrafficLightState)payload->state); protocol_send_response(uart_id, CMD_SET_LIGHT, RESP_OK, NULL, 0U); }
            }
            break;
        case CMD_EMERGENCY_TRIGGER:
        case CMD_EMERGENCY_CLEAR:
            if (frame->length < sizeof(EmergencyPayload)) protocol_send_error(uart_id, RESP_INVALID_PARAM);
            else {
                const EmergencyPayload *payload = (const EmergencyPayload *)frame->payload;
                if (payload->direction >= NUM_DIRECTIONS) protocol_send_error(uart_id, RESP_INVALID_PARAM);
                else { if (frame->cmd == CMD_EMERGENCY_TRIGGER) activate_emergency_locked((Direction)payload->direction); else clear_emergency_locked((Direction)payload->direction); protocol_send_response(uart_id, frame->cmd, RESP_OK, NULL, 0U); }
            }
            break;
        case CMD_PEDESTRIAN_REQUEST:
            if (frame->length < 1U || frame->payload[0] >= NUM_DIRECTIONS) protocol_send_error(uart_id, RESP_INVALID_PARAM);
            else { g_system.lanes[frame->payload[0]].pedestrian_waiting++; protocol_send_response(uart_id, CMD_PEDESTRIAN_REQUEST, RESP_OK, NULL, 0U); }
            break;
        default:
            protocol_send_error(uart_id, RESP_NOT_SUPPORTED);
            break;
    }
    pthread_mutex_unlock(&system_lock);
}

static void *communication_task(void *unused) {
    (void)unused;
    uint8_t raw[PROTO_MAX_FRAME_SIZE];
    uint16_t used = 0U;
    protocol_init();
    hal_uart_init(0U, UART_BAUD_115200, UART_PARITY_NONE, UART_STOP_1);
    while (system_running) {
        uint8_t byte;
        wdg_pet(TASK_COMM);
        if (hal_uart_receive(0U, &byte, 1U, 10U) > 0) {
            if (used == 0U && byte != PROTO_SYNC_BYTE_1) continue;
            raw[used++] = byte;
            if (used == 2U && raw[1] != PROTO_SYNC_BYTE_2) used = 0U;
            if (used >= PROTO_HEADER_SIZE) {
                uint16_t payload_length = ((uint16_t)raw[2] << 8) | raw[3];
                if (payload_length > PROTO_MAX_PAYLOAD) used = 0U;
                else if (used == PROTO_HEADER_SIZE + payload_length + PROTO_CRC_SIZE) {
                    ProtocolFrame frame;
                    if (protocol_decode_frame(raw, used, &frame) == 0) process_frame(0U, &frame);
                    used = 0U;
                }
            }
        }
        hal_delay_ms(50U);
    }
    return NULL;
}

static void *diagnostics_task(void *unused) {
    (void)unused;
    while (system_running) {
        wdg_pet(TASK_DIAGNOSTICS);
        pthread_mutex_lock(&system_lock);
        for (Direction d = DIR_NORTH; d < NUM_DIRECTIONS; d++) {
            for (uint32_t s = 0U; s < MAX_SENSORS_PER_LANE; s++) {
                if (g_system.sensors[d][s].confidence < 30U) wdg_record_fault(FAULT_SENSOR_FAILURE, TASK_DIAGNOSTICS, g_system.sensors[d][s].id, "Sensor confidence low");
            }
        }
        pthread_mutex_unlock(&system_lock);
        hal_delay_ms(1000U);
    }
    return NULL;
}

static void *simulation_task(void *unused) {
    (void)unused;
    while (system_running) {
        wdg_pet(TASK_SIMULATION);
        pthread_mutex_lock(&system_lock);
        if ((rand() % 100) < 25) {
            Direction direction = (Direction)(rand() % NUM_DIRECTIONS);
            VehicleType type = (rand() % 100 < 2) ? VEHICLE_EMERGENCY : VEHICLE_CAR;
            if (enqueue_vehicle_locked(direction, type, type == VEHICLE_EMERGENCY ? EMERGENCY_PRIORITY : NORMAL_PRIORITY) && type == VEHICLE_EMERGENCY) activate_emergency_locked(direction);
        }
        if ((rand() % 100) < 10) g_system.lanes[rand() % NUM_DIRECTIONS].pedestrian_waiting++;
        for (Direction d = DIR_NORTH; d < NUM_DIRECTIONS; d++) {
            if (g_system.lanes[d].emergency_override && (rand() % 1000) < 5) clear_emergency_locked(d);
        }
        pthread_mutex_unlock(&system_lock);
        hal_delay_ms(100U);
    }
    return NULL;
}

static void signal_handler(int signal_number) {
    (void)signal_number;
    system_running = 0;
    g_running = false;
}

int main(void) {
    pthread_t sensor, control, communication, diagnostics, simulation;
    srand((unsigned int)time(NULL));
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    memset(&g_system, 0, sizeof(g_system));
    gettimeofday(&g_system.start_time, NULL);
    g_system.current_mode = MODE_NORMAL;
    g_system.previous_mode = MODE_NORMAL;
    g_system.system_efficiency = 1.0f;
    for (Direction d = DIR_NORTH; d < NUM_DIRECTIONS; d++) {
        g_system.lanes[d].direction = d;
        g_system.lanes[d].green_duration = MIN_GREEN_TIME;
        for (uint32_t s = 0U; s < MAX_SENSORS_PER_LANE; s++) {
            Sensor *sensor_data = &g_system.sensors[d][s];
            sensor_data->id = (uint32_t)d * 100U + s;
            sensor_data->direction = d;
            sensor_data->type = (SensorType)s;
            sensor_data->active = true;
            sensor_data->confidence = 95U;
        }
    }
    for (uint32_t pin = TRAFFIC_NORTH_RED; pin <= TRAFFIC_WEST_GREEN; pin++) hal_gpio_init(pin, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    wdg_init(WDG_TIMEOUT_MS);
    wdg_register_task(TASK_SENSOR, "sensor", SENSOR_SAMPLE_RATE_MS, true);
    wdg_register_task(TASK_CONTROL, "control", CONTROL_TICK_MS, true);
    wdg_register_task(TASK_COMM, "comm", 50U, true);
    wdg_register_task(TASK_DIAGNOSTICS, "diagnostics", 1000U, false);
    wdg_register_task(TASK_SIMULATION, "simulation", 100U, false);
    log_event("SYSTEM", "POSIX traffic simulation started; press Ctrl+C to stop");
    pthread_create(&sensor, NULL, sensor_task, NULL);
    pthread_create(&control, NULL, control_task, NULL);
    pthread_create(&communication, NULL, communication_task, NULL);
    pthread_create(&diagnostics, NULL, diagnostics_task, NULL);
    pthread_create(&simulation, NULL, simulation_task, NULL);
    pthread_join(sensor, NULL);
    pthread_join(control, NULL);
    pthread_join(communication, NULL);
    pthread_join(diagnostics, NULL);
    pthread_join(simulation, NULL);
    pthread_mutex_lock(&system_lock);
    set_all_red_locked();
    pthread_mutex_unlock(&system_lock);
    log_event("SYSTEM", "Simulation stopped safely");
    return 0;
}