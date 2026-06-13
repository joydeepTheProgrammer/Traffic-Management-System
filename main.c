/**
 * @file main.c
 * @brief Traffic Management System - Firmware Entry Point
 * @version 2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include "traffic_system.h"
#include "hal.h"
#include "scheduler.h"
#include "protocol.h"
#include "watchdog.h"

/* ============================================================================
 * TASK DEFINITIONS
 * ============================================================================ */
#define TASK_SENSOR         1
#define TASK_CONTROL        2
#define TASK_COMM           3
#define TASK_DISPLAY        4
#define TASK_DIAGNOSTICS    5
#define TASK_SIMULATION     6

/* ============================================================================
 * GLOBAL STATE
 * ============================================================================ */
static volatile bool system_running = true;
static SimpleMutex g_system_mutex;

/* ============================================================================
 * TASK FUNCTIONS
 * ============================================================================ */

static void task_sensor(void *arg) {
    (void)arg;
    log_event("TASK", "Sensor task started (period: 500ms)");

    while (system_running) {
        wdg_pet(TASK_SENSOR);

        mutex_lock(&g_system_mutex, 100);

        /* Read all sensors */
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            for (int s = 0; s < MAX_SENSORS_PER_LANE; s++) {
                Sensor *sensor = &g_system.sensors[d][s];
                if (!sensor->active) continue;

                /* Simulate sensor reading based on actual HW or simulation */
                float value = 0.0f;
                switch (sensor->type) {
                    case SENSOR_IR:
                        value = hal_gpio_read(SENSOR_IR_NORTH + d * 2) ? 1.0f : 0.0f;
                        break;
                    case SENSOR_ULTRASONIC:
                        /* Trigger ultrasonic and measure echo */
                        hal_gpio_write(SENSOR_US_TRIG_NORTH + d * 4, true);
                        hal_delay_ms(1);
                        hal_gpio_write(SENSOR_US_TRIG_NORTH + d * 4, false);
                        /* In real HW: measure pulse width on echo pin */
                        value = 150.0f; /* Simulated distance in cm */
                        break;
                    case SENSOR_INDUCTIVE:
                        value = hal_adc_read_voltage(ADC_CHANNEL_0 + d);
                        break;
                    case SENSOR_CAMERA:
                        /* AI vision processing - simulated count */
                        value = (float)g_system.lanes[d].vehicle_count;
                        break;
                    case SENSOR_V2X:
                        /* V2X beacon detection */
                        value = (g_system.lanes[d].vehicle_count > 0) ? 1.0f : 0.0f;
                        break;
                    case SENSOR_WEATHER:
                        value = hal_adc_read_voltage(ADC_CHANNEL_3);
                        break;
                    default:
                        value = 0.0f;
                }

                sensor->last_reading = value;
                gettimeofday(&sensor->last_update, NULL);

                /* Sensor health monitoring */
                if (sensor->confidence > 0 && (rand() % 1000) < 5) {
                    sensor->confidence--;
                }
            }
        }

        mutex_unlock(&g_system_mutex);
        hal_delay_ms(SENSOR_SAMPLE_RATE_MS);
    }

    log_event("TASK", "Sensor task stopped");
}

static void task_control(void *arg) {
    (void)arg;
    log_event("TASK", "Control task started (period: 100ms)");

    static Direction current_green = DIR_NORTH;
    static uint32_t phase_step = 0;
    static uint32_t tick_counter = 0;
    static uint32_t status_counter = 0;

    while (system_running) {
        wdg_pet(TASK_CONTROL);

        mutex_lock(&g_system_mutex, 100);

        /* Check night mode */
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        bool is_night = (tm_info->tm_hour >= 22 || tm_info->tm_hour < 5);

        if (is_night && g_system.current_mode == MODE_NORMAL) {
            g_system.current_mode = MODE_NIGHT;
            log_event("MODE", "Night mode activated");
        } else if (!is_night && g_system.current_mode == MODE_NIGHT) {
            g_system.current_mode = MODE_NORMAL;
            log_event("MODE", "Day mode resumed");
        }

        /* Check congestion */
        uint32_t total_waiting = 0;
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            total_waiting += g_system.lanes[d].vehicle_count;
        }
        float system_load = (float)total_waiting / (NUM_DIRECTIONS * MAX_QUEUE_LENGTH);

        if (system_load > (CONGESTION_THRESHOLD / 100.0f) && 
            g_system.current_mode != MODE_EMERGENCY) {
            if (g_system.current_mode != MODE_CONGESTION) {
                g_system.previous_mode = g_system.current_mode;
                g_system.current_mode = MODE_CONGESTION;
                log_event("MODE", "Congestion mode activated");
            }
        } else if (g_system.current_mode == MODE_CONGESTION) {
            g_system.current_mode = g_system.previous_mode;
            log_event("MODE", "Congestion cleared");
        }

        /* Emergency mode handling */
        if (g_system.current_mode == MODE_EMERGENCY) {
            for (int d = 0; d < NUM_DIRECTIONS; d++) {
                uint32_t pin_base = TRAFFIC_NORTH_RED + d * 3;
                if (g_system.lanes[d].emergency_override) {
                    hal_gpio_write(pin_base + 2, true);   /* GREEN */
                    hal_gpio_write(pin_base, false);        /* RED */
                    hal_gpio_write(pin_base + 1, false);    /* YELLOW */
                } else {
                    hal_gpio_write(pin_base, true);         /* RED */
                    hal_gpio_write(pin_base + 1, false);   /* YELLOW */
                    hal_gpio_write(pin_base + 2, false);     /* GREEN */
                }
            }
            mutex_unlock(&g_system_mutex);
            hal_delay_ms(SYSTEM_TICK_MS);
            continue;
        }

        /* Night mode - blinking yellow */
        if (g_system.current_mode == MODE_NIGHT) {
            static bool blink_state = false;
            blink_state = !blink_state;
            for (int d = 0; d < NUM_DIRECTIONS; d++) {
                uint32_t pin_base = TRAFFIC_NORTH_RED + d * 3;
                hal_gpio_write(pin_base, false);
                hal_gpio_write(pin_base + 1, blink_state);
                hal_gpio_write(pin_base + 2, false);
            }
            mutex_unlock(&g_system_mutex);
            hal_delay_ms(SYSTEM_TICK_MS);
            continue;
        }

        /* Normal phase state machine */
        LaneController *current = &g_system.lanes[current_green];
        uint32_t pin_base = TRAFFIC_NORTH_RED + current_green * 3;

        switch (phase_step) {
            case 0: /* GREEN */
                hal_gpio_write(pin_base, false);
                hal_gpio_write(pin_base + 1, false);
                hal_gpio_write(pin_base + 2, true);

                /* Set other directions to RED */
                for (int d = 0; d < NUM_DIRECTIONS; d++) {
                    if (d != (int)current_green) {
                        uint32_t other_base = TRAFFIC_NORTH_RED + d * 3;
                        hal_gpio_write(other_base, true);
                        hal_gpio_write(other_base + 1, false);
                        hal_gpio_write(other_base + 2, false);
                    }
                }

                tick_counter++;
                if (tick_counter >= current->green_duration) {
                    tick_counter = 0;
                    phase_step = 1;
                    /* Process vehicles */
                    uint32_t processed = current->vehicle_count / 3 + 1;
                    for (uint32_t i = 0; i < processed && current->vehicle_count > 0; i++) {
                        current->vehicle_count--;
                        g_system.total_vehicles_processed++;
                    }
                }
                break;

            case 1: /* YELLOW */
                hal_gpio_write(pin_base, false);
                hal_gpio_write(pin_base + 1, true);
                hal_gpio_write(pin_base + 2, false);

                tick_counter++;
                if (tick_counter >= YELLOW_TIME) {
                    tick_counter = 0;
                    phase_step = 2;
                }
                break;

            case 2: /* ALL RED */
                hal_gpio_write(pin_base, true);
                hal_gpio_write(pin_base + 1, false);
                hal_gpio_write(pin_base + 2, false);

                tick_counter++;
                if (tick_counter >= ALL_RED_TIME) {
                    tick_counter = 0;
                    phase_step = 3;
                }
                break;

            case 3: /* SELECT NEXT */
                /* Update density history */
                current->density_history[current->history_index] = 
                    (float)current->vehicle_count / MAX_QUEUE_LENGTH;
                current->history_index = (current->history_index + 1) % HISTORY_SIZE;

                /* Calculate adaptive green time */
                float avg_density = calculate_moving_average(current);
                uint32_t new_green = MIN_GREEN_TIME;
                float density = current->density_history[(current->history_index + HISTORY_SIZE - 1) % HISTORY_SIZE];

                if (density > 0.3f) {
                    new_green += (uint32_t)((density - 0.3f) * 40.0f);
                }
                if (avg_density > density) {
                    new_green += 5;
                }
                if (current->pedestrian_waiting > 0) {
                    new_green += (current->pedestrian_waiting > 5) ? 10 : 5;
                }
                if (new_green > MAX_GREEN_TIME) new_green = MAX_GREEN_TIME;
                if (new_green < MIN_GREEN_TIME) new_green = MIN_GREEN_TIME;
                current->green_duration = new_green;

                /* Select next direction by priority */
                Direction next_dir = (Direction)((current_green + 1) % NUM_DIRECTIONS);
                uint32_t max_priority = 0;

                for (int d = 0; d < NUM_DIRECTIONS; d++) {
                    if ((Direction)d == current_green) continue;
                    uint32_t priority = g_system.lanes[d].vehicle_count + 
                                       g_system.lanes[d].pedestrian_waiting * 2;
                    if (g_system.lanes[d].emergency_override) priority += 1000;
                    if (priority > max_priority) {
                        max_priority = priority;
                        next_dir = (Direction)d;
                    }
                }

                current_green = next_dir;
                current->cycle_count++;
                phase_step = 0;

                log_event("PHASE", "Next: %s (green: %us, vehicles: %u)",
                          direction_to_str(current_green),
                          g_system.lanes[current_green].green_duration,
                          g_system.lanes[current_green].vehicle_count);
                break;
        }

        /* Process pedestrians during green */
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            if (g_system.lanes[d].state == LIGHT_GREEN && 
                g_system.lanes[d].pedestrian_waiting > 0) {
                g_system.lanes[d].pedestrian_waiting--;
                g_system.total_pedestrians_processed++;
            }
        }

        /* Update light state tracking */
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            uint32_t base = TRAFFIC_NORTH_RED + d * 3;
            if (hal_gpio_read(base + 2)) {
                g_system.lanes[d].state = LIGHT_GREEN;
            } else if (hal_gpio_read(base + 1)) {
                g_system.lanes[d].state = LIGHT_YELLOW;
            } else if (hal_gpio_read(base)) {
                g_system.lanes[d].state = LIGHT_RED;
            }
        }

        /* Periodic status print */
        status_counter++;
        if (status_counter >= 50) {
            printf("\n=== STATUS === Mode: %s | Queue: %u/%u | Veh: %u | Ped: %u\n",
                   mode_to_str(g_system.current_mode),
                   g_system.queue_count, MAX_QUEUE_LENGTH,
                   g_system.total_vehicles_processed,
                   g_system.total_pedestrians_processed);
            for (int d = 0; d < NUM_DIRECTIONS; d++) {
                printf("  %s: %s | V:%2u P:%2u | Green:%3us\n",
                       direction_to_str((Direction)d),
                       light_to_str(g_system.lanes[d].state),
                       g_system.lanes[d].vehicle_count,
                       g_system.lanes[d].pedestrian_waiting,
                       g_system.lanes[d].green_duration);
            }
            status_counter = 0;
        }

        mutex_unlock(&g_system_mutex);
        hal_delay_ms(SYSTEM_TICK_MS);
    }

    log_event("TASK", "Control task stopped");
}

static void task_communication(void *arg) {
    (void)arg;
    log_event("TASK", "Communication task started (period: 50ms)");

    uint8_t uart_id = 0;
    hal_uart_init(uart_id, UART_BAUD_115200, UART_PARITY_NONE, UART_STOP_1);
    protocol_init();

    ProtocolFrame rx_frame;
    uint8_t rx_buffer[PROTO_MAX_FRAME_SIZE];
    uint16_t rx_index = 0;

    while (system_running) {
        wdg_pet(TASK_COMM);

        /* Receive bytes */
        uint8_t byte;
        int ret = hal_uart_receive(uart_id, &byte, 1, 10);
        if (ret > 0) {
            rx_buffer[rx_index++] = byte;

            /* Check for complete frame */
            if (rx_index >= PROTO_HEADER_SIZE && 
                rx_buffer[0] == PROTO_SYNC_BYTE_1 && 
                rx_buffer[1] == PROTO_SYNC_BYTE_2) {
                uint16_t payload_len = (rx_buffer[2] << 8) | rx_buffer[3];
                if (rx_index >= PROTO_HEADER_SIZE + payload_len + PROTO_CRC_SIZE) {
                    /* Decode and process frame */
                    if (protocol_decode_frame(rx_buffer, rx_index, &rx_frame) == 0) {
                        mutex_lock(&g_system_mutex, 100);

                        switch (rx_frame.cmd) {
                            case CMD_HEARTBEAT:
                                protocol_send_response(uart_id, CMD_HEARTBEAT, RESP_OK, NULL, 0);
                                break;

                            case CMD_STATUS_REQ:
                                protocol_send_status(uart_id);
                                break;

                            case CMD_SET_LIGHT: {
                                LightControlPayload *p = (LightControlPayload*)rx_frame.payload;
                                if (p->direction < NUM_DIRECTIONS) {
                                    uint32_t base = TRAFFIC_NORTH_RED + p->direction * 3;
                                    hal_gpio_write(base, p->state == LIGHT_RED);
                                    hal_gpio_write(base + 1, p->state == LIGHT_YELLOW);
                                    hal_gpio_write(base + 2, p->state == LIGHT_GREEN);
                                    protocol_send_response(uart_id, CMD_SET_LIGHT, RESP_OK, NULL, 0);
                                }
                                break;
                            }

                            case CMD_EMERGENCY_TRIGGER: {
                                EmergencyPayload *p = (EmergencyPayload*)rx_frame.payload;
                                if (p->direction < NUM_DIRECTIONS) {
                                    g_system.lanes[p->direction].emergency_override = true;
                                    g_system.emergency_count++;
                                    g_system.previous_mode = g_system.current_mode;
                                    g_system.current_mode = MODE_EMERGENCY;
                                    protocol_send_response(uart_id, CMD_EMERGENCY_TRIGGER, RESP_OK, NULL, 0);
                                    log_event("COMM", "Emergency triggered from %s", 
                                              direction_to_str((Direction)p->direction));
                                }
                                break;
                            }

                            case CMD_EMERGENCY_CLEAR: {
                                EmergencyPayload *p = (EmergencyPayload*)rx_frame.payload;
                                if (p->direction < NUM_DIRECTIONS) {
                                    g_system.lanes[p->direction].emergency_override = false;
                                    if (g_system.emergency_count > 0) g_system.emergency_count--;
                                    if (g_system.emergency_count == 0) {
                                        g_system.current_mode = g_system.previous_mode;
                                    }
                                    protocol_send_response(uart_id, CMD_EMERGENCY_CLEAR, RESP_OK, NULL, 0);
                                }
                                break;
                            }

                            case CMD_PEDESTRIAN_REQUEST: {
                                uint8_t dir = rx_frame.payload[0];
                                if (dir < NUM_DIRECTIONS) {
                                    g_system.lanes[dir].pedestrian_waiting++;
                                    protocol_send_response(uart_id, CMD_PEDESTRIAN_REQUEST, RESP_OK, NULL, 0);
                                }
                                break;
                            }

                            case CMD_RESET:
                                protocol_send_response(uart_id, CMD_RESET, RESP_OK, NULL, 0);
                                hal_delay_ms(100);
                                hal_system_reset();
                                break;

                            default:
                                protocol_send_error(uart_id, RESP_INVALID_CMD);
                                break;
                        }

                        mutex_unlock(&g_system_mutex);
                    }
                    rx_index = 0;
                }
            }

            if (rx_index >= PROTO_MAX_FRAME_SIZE) {
                rx_index = 0; /* Overflow reset */
            }
        }

        hal_delay_ms(50);
    }

    log_event("TASK", "Communication task stopped");
}

static void task_diagnostics(void *arg) {
    (void)arg;
    log_event("TASK", "Diagnostics task started (period: 1000ms)");

    while (system_running) {
        wdg_pet(TASK_DIAGNOSTICS);

        mutex_lock(&g_system_mutex, 100);

        /* Check sensor health */
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            for (int s = 0; s < MAX_SENSORS_PER_LANE; s++) {
                Sensor *sensor = &g_system.sensors[d][s];
                if (sensor->confidence < 30) {
                    wdg_record_fault(FAULT_SENSOR_FAILURE, TASK_DIAGNOSTICS,
                                    sensor->id, "Sensor confidence low");
                }
            }
        }

        /* Check power */
        float voltage, current;
        hal_power_get_voltage(&voltage);
        hal_power_get_current(&current);

        if (voltage < 10.0f) {
            wdg_record_fault(FAULT_POWER_LOW, TASK_DIAGNOSTICS, 0, "Low supply voltage");
        }

        /* Check temperature */
        float temp = hal_adc_read_voltage(ADC_CHANNEL_3);
        if (temp > 80.0f) {
            wdg_record_fault(FAULT_OVER_TEMPERATURE, TASK_DIAGNOSTICS, 
                            (uint32_t)temp, "Over temperature");
        }

        /* Update system health */
        SystemHealth health;
        wdg_get_health(&health);
        health.supply_voltage = voltage;
        health.supply_current = current;
        health.cpu_temperature = temp;

        /* Calculate efficiency */
        uint32_t total_waiting = 0;
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            total_waiting += g_system.lanes[d].vehicle_count;
        }
        g_system.system_efficiency = 1.0f - ((float)total_waiting / (NUM_DIRECTIONS * MAX_QUEUE_LENGTH));

        mutex_unlock(&g_system_mutex);

        hal_delay_ms(1000);
    }

    log_event("TASK", "Diagnostics task stopped");
}

static void task_simulation(void *arg) {
    (void)arg;
    log_event("TASK", "Simulation task started (period: 100ms)");

    uint32_t vehicle_counter = 0;

    while (system_running) {
        wdg_pet(TASK_SIMULATION);

        mutex_lock(&g_system_mutex, 100);

        /* Simulate vehicle arrivals */
        if ((rand() % 100) < 35) {
            Direction dir = (Direction)(rand() % NUM_DIRECTIONS);
            VehicleType type = VEHICLE_CAR;
            uint32_t priority = NORMAL_PRIORITY;

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

            if (g_system.queue_count < MAX_QUEUE_LENGTH) {
                VehicleQueue *vq = &g_system.queue[g_system.queue_tail];
                vq->vehicle_id = ++vehicle_counter;
                vq->type = type;
                vq->approach = dir;
                vq->priority = priority;
                gettimeofday(&vq->arrival_time, NULL);
                vq->processed = false;

                g_system.queue_tail = (g_system.queue_tail + 1) % MAX_QUEUE_LENGTH;
                g_system.queue_count++;
                g_system.lanes[dir].vehicle_count++;

                if (type == VEHICLE_EMERGENCY) {
                    g_system.lanes[dir].emergency_override = true;
                    g_system.emergency_count++;
                    g_system.previous_mode = g_system.current_mode;
                    g_system.current_mode = MODE_EMERGENCY;
                    log_event("SIM", "Emergency vehicle %u from %s", 
                              vq->vehicle_id, direction_to_str(dir));
                }
            }
        }

        /* Simulate pedestrian requests */
        if ((rand() % 100) < 12) {
            Direction dir = (Direction)(rand() % NUM_DIRECTIONS);
            g_system.lanes[dir].pedestrian_waiting++;
        }

        /* Clear emergency after some time */
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            if (g_system.lanes[d].emergency_override && (rand() % 1000) < 5) {
                g_system.lanes[d].emergency_override = false;
                if (g_system.emergency_count > 0) g_system.emergency_count--;
                if (g_system.emergency_count == 0) {
                    g_system.current_mode = g_system.previous_mode;
                    log_event("SIM", "Emergency cleared from %s", direction_to_str((Direction)d));
                }
            }
        }

        mutex_unlock(&g_system_mutex);
        hal_delay_ms(100);
    }

    log_event("TASK", "Simulation task stopped");
}

/* ============================================================================
 * SIGNAL HANDLER
 * ============================================================================ */
static void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        log_event("SIGNAL", "Shutdown signal received: %d", sig);
        system_running = false;
    }
}

/* ============================================================================
 * MAIN
 * ============================================================================ */
int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════════════╗\n");
    printf("║     TRAFFIC MANAGEMENT SYSTEM - EMBEDDED FIRMWARE v2.0             ║\n");
    printf("║     Target: ARM Cortex-M4 / POSIX / FreeRTOS                       ║\n");
    printf("╚══════════════════════════════════════════════════════════════════════╝\n\n");

    srand((unsigned int)time(NULL));
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Initialize HAL */
    hal_gpio_init(TRAFFIC_NORTH_RED, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_NORTH_YELLOW, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_NORTH_GREEN, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_SOUTH_RED, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_SOUTH_YELLOW, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_SOUTH_GREEN, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_EAST_RED, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_EAST_YELLOW, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_EAST_GREEN, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_WEST_RED, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_WEST_YELLOW, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);
    hal_gpio_init(TRAFFIC_WEST_GREEN, GPIO_MODE_OUTPUT, GPIO_SPEED_HIGH);

    /* Initialize system */
    memset(&g_system, 0, sizeof(TrafficSystem));
    g_system.current_mode = MODE_NORMAL;
    g_system.previous_mode = MODE_NORMAL;
    g_system.system_efficiency = 1.0f;
    gettimeofday(&g_system.start_time, NULL);

    /* Initialize sensors */
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
    g_system.sensors[DIR_NORTH][2].type = SENSOR_CAMERA;
    g_system.sensors[DIR_SOUTH][2].type = SENSOR_CAMERA;
    g_system.sensors[DIR_EAST][2].type = SENSOR_V2X;
    g_system.sensors[DIR_WEST][2].type = SENSOR_V2X;

    /* Initialize lane controllers */
    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        g_system.lanes[d].direction = (Direction)d;
        g_system.lanes[d].state = LIGHT_RED;
        g_system.lanes[d].green_duration = MIN_GREEN_TIME;
    }

    /* Initialize subsystems */
    mutex_init(&g_system_mutex);
    scheduler_init();
    wdg_init(WDG_TIMEOUT_MS);

    /* Register tasks with watchdog */
    wdg_register_task(TASK_SENSOR, "sensor", 500, true);
    wdg_register_task(TASK_CONTROL, "control", 100, true);
    wdg_register_task(TASK_COMM, "comm", 50, true);
    wdg_register_task(TASK_DIAGNOSTICS, "diagnostics", 1000, false);
    wdg_register_task(TASK_SIMULATION, "simulation", 100, false);

    /* Create tasks */
    scheduler_create_task("sensor", task_sensor, NULL, PRIO_HIGH, 500, true);
    scheduler_create_task("control", task_control, NULL, PRIO_CRITICAL, 100, true);
    scheduler_create_task("comm", task_communication, NULL, PRIO_HIGH, 50, true);
    scheduler_create_task("diagnostics", task_diagnostics, NULL, PRIO_NORMAL, 1000, true);
    scheduler_create_task("simulation", task_simulation, NULL, PRIO_LOW, 100, true);

    log_event("MAIN", "System initialized - starting scheduler");
    log_event("MAIN", "Press Ctrl+C to shutdown\n");

    /* Run scheduler (cooperative) */
    scheduler_run();

    /* Shutdown */
    log_event("MAIN", "System shutting down...");

    /* All lights RED */
    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        uint32_t base = TRAFFIC_NORTH_RED + d * 3;
        hal_gpio_write(base, true);
        hal_gpio_write(base + 1, false);
        hal_gpio_write(base + 2, false);
    }

    printf("\nSystem shutdown complete.\n");
    return 0;
}
