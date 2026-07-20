/**
 * @file main.c
 * @brief Main firmware – Integrated Intelligent Vehicle Safety and Accident Prevention System
 * @author Vehicle Safety System Project
 * @version 1.0.0
 *
 * @target  ATmega328P @ 16 MHz
 * @toolchain AVR-GCC + avrdude
 *
 * ════════════════════════════════════════════════════════════════════════════
 *  SYSTEM OVERVIEW
 * ════════════════════════════════════════════════════════════════════════════
 *
 *  This firmware orchestrates 8 safety subsystems:
 *
 *  1. COLLISION AVOIDANCE   – 4× HC-SR04 ultrasonic sensors, brake relay
 *  2. DRUNK DRIVER DETECT   – MQ-3 alcohol sensor, engine cut-off relay
 *  3. CRASH / ROLLOVER      – MPU-6050 IMU, emergency SMS via SIM800L
 *  4. LANE DEPARTURE        – 2× IR sensors, audible warning
 *  5. DRIVER DROWSINESS     – GPS speed + time heuristic, audible warning
 *  6. AUTO HEADLIGHTS       – LDR sensor, headlight relay
 *  7. CABIN OVERHEAT        – DHT11 temperature sensor, evacuation alarm
 *  8. GPS EMERGENCY BEACON  – NEO-6M GPS + SIM800L GSM on crash/alcohol
 *
 * ════════════════════════════════════════════════════════════════════════════
 *  PIN MAP (ATmega328P)
 * ════════════════════════════════════════════════════════════════════════════
 *
 *  Port B:
 *    PB0 – US FRONT TRIG       PB1 – US FRONT ECHO
 *    PB2 – US REAR  TRIG       PB3 – US REAR  ECHO
 *    PB4 – Headlight Relay     PB5 – Buzzer
 *    PB6 – LED Collision       PB7 – LED Alcohol
 *
 *  Port C (analog side):
 *    PC0 – US LEFT  TRIG       PC1 – US LEFT  ECHO
 *    PC2 – US RIGHT TRIG       PC3 – US RIGHT ECHO
 *    PC4 – LDR ADC (ADC4)      PC5 – MQ-3 ADC (ADC5)
 *    PC4 – SDA (I2C) *shared*  PC5 – SCL (I2C) *shared*
 *
 *    NOTE: PC4/PC5 serve dual purpose.  Ultrasonic LEFT/RIGHT are on PC0-PC3.
 *    I2C (TWI hardware) and ADC4/ADC5 can coexist because TWI uses the SDA/SCL
 *    alternate functions and the LDR/MQ3 readings are taken outside I2C transactions.
 *
 *  Port D:
 *    PD0 – Engine Cut-off Relay  PD1 – Brake Assist Relay
 *    PD2 – GSM Power Key         PD3 – GSM TX (soft UART)
 *    PD4 – GSM RX (soft UART)    PD5 – GPS TX (soft UART) [to GPS RX]
 *    PD6 – GPS RX (soft UART)    PD7 – DHT11 Data
 *    PD3 – LED Crash (conflicts with GSM TX; LED on PD3 removed, placed on PD8 if expanded)
 *
 *  Port C (digital):
 *    PC4 – IR Left sensor         PC5 – IR Right sensor (digital mode)
 *
 *    DESIGN NOTE: PC4/PC5 used as digital IR inputs when I2C is idle and
 *    ADC is not sampling.  In practice these sensors are read at different
 *    phases in the loop.  For production, dedicate separate I/O expander pins
 *    via PCF8574 for IR and use onboard ADC for LDR/MQ3 only.
 *
 * ════════════════════════════════════════════════════════════════════════════
 *  TIMING
 * ════════════════════════════════════════════════════════════════════════════
 *
 *  Timer0: 1 ms overflow ISR – system millisecond counter (g_system_ms).
 *  Timer1: Free-running, used by ultrasonic driver for echo timing.
 *
 * ════════════════════════════════════════════════════════════════════════════
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>

/* ── Driver includes ─────────────────────────────────────────────────────── */
#include "uart.h"
#include "lcd_i2c.h"
#include "gsm_sim800l.h"
#include "gps_neo6m.h"

/* ── Sensor includes ─────────────────────────────────────────────────────── */
#include "ultrasonic.h"
#include "mpu6050.h"
#include "mq3.h"
#include "dht11.h"
#include "ir_sensor.h"
#include "ldr.h"

/* ── Safety module includes ──────────────────────────────────────────────── */
#include "collision.h"
#include "alerts.h"

/* ════════════════════════════════════════════════════════════════════════════
 *  Global system millisecond counter
 *  Incremented by Timer0 Compare Match A ISR every 1 ms.
 * ════════════════════════════════════════════════════════════════════════════ */
volatile uint32_t g_system_ms = 0;

ISR(TIMER0_COMPA_vect)
{
    g_system_ms++;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Timer0 initialisation: CTC mode, 1 ms period at 16 MHz
 *  OCR0A = (F_CPU / (prescaler × 1000)) - 1 = (16000000 / (64 × 1000)) - 1 = 249
 * ════════════════════════════════════════════════════════════════════════════ */
static void timer0_ms_init(void)
{
    TCCR0A = (1U << WGM01);           /* CTC mode                         */
    TCCR0B = (1U << CS01) | (1U << CS00); /* Prescaler = 64               */
    OCR0A  = 249U;                    /* Compare value for 1 ms period     */
    TIMSK0 = (1U << OCIE0A);          /* Enable compare match A interrupt  */
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Drowsiness detection state
 * ════════════════════════════════════════════════════════════════════════════ */
#define DROWSY_CHECK_INTERVAL_MS   30000UL  /**< Check every 30 seconds      */
#define DROWSY_SPEED_STABLE_KMH    5.0f     /**< Max speed variance allowed  */
#define DROWSY_MIN_DRIVE_SPEED_KMH 15.0f   /**< Below this: not driving     */

typedef struct {
    float    last_speed_kmh;
    uint32_t last_check_ms;
    uint8_t  alert_count;           /**< Consecutive drowsy alerts       */
} DrowsyState_t;

/* ════════════════════════════════════════════════════════════════════════════
 *  LCD display helper – shows 2 lines, max 16 chars each
 * ════════════════════════════════════════════════════════════════════════════ */
static void lcd_show(const char *line1, const char *line2)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print_str(line1);
    lcd_set_cursor(0, 1);
    lcd_print_str(line2);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Crash/Alcohol emergency handler – sends SMS and engages safety systems
 * ════════════════════════════════════════════════════════════════════════════ */
static void handle_emergency(const char *reason, float lat, float lon,
                              uint8_t cut_engine)
{
    /* Engage actuators immediately */
    if (cut_engine) {
        engine_relay_cut();
    }
    brake_relay_engage();

    /* Flash crash LED and sound critical alarm */
    for (uint8_t i = 0; i < 5U; i++) {
        buzzer_beep(ALERT_CRITICAL);
    }

    /* Display on LCD */
    lcd_show("!! EMERGENCY !!", reason);

    /* Log to serial (debug) */
    uart_puts("EMERGENCY: ");
    uart_puts(reason);
    uart_puts("\r\n");

    /* Send emergency SMS with GPS location */
    gsm_send_emergency_sms(lat, lon, reason);

    /* Wait 5 seconds before resuming main loop */
    _delay_ms(5000);

    /* Release brake (engine cut stays until manual reset or restart) */
    brake_relay_release();
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Collision display helper
 * ════════════════════════════════════════════════════════════════════════════ */
static void display_collision_info(const CollisionData_t *col)
{
    static const char dir_chars[US_SENSOR_COUNT] = {'F', 'R', 'L', 'r'};
    char line[17] = "Col:            ";

    for (uint8_t i = 0; i < US_SENSOR_COUNT; i++) {
        if (col->direction_flags & (1U << i)) {
            line[4 + i * 2]     = dir_chars[i];
            if (col->dist_cm[i] < 100U) {
                line[5 + i * 2] = (char)('0' + col->dist_cm[i] / 10U);
            } else {
                line[5 + i * 2] = '!';
            }
        }
    }
    lcd_show("Collision Warn!", line);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Normal status display (speed + temp)
 * ════════════════════════════════════════════════════════════════════════════ */
static void display_normal_status(float speed_kmh, uint8_t temp_c)
{
    /* Line 1: "Speed: XXX km/h " */
    char line1[17] = "Speed:     km/h ";
    int16_t sp = (int16_t)speed_kmh;
    if (sp < 0) sp = 0;
    line1[7]  = (char)('0' + (sp / 100) % 10);
    line1[8]  = (char)('0' + (sp / 10)  % 10);
    line1[9]  = (char)('0' + (sp)       % 10);

    /* Line 2: "Temp:  XX C OK  " */
    char line2[17] = "Temp:  XX C     ";
    line2[7] = (char)('0' + (temp_c / 10) % 10);
    line2[8] = (char)('0' + (temp_c)      % 10);

    lcd_show(line1, line2);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  System initialisation
 * ════════════════════════════════════════════════════════════════════════════ */
static void system_init(void)
{
    /* Disable watchdog (just in case of reset loop) */
    /* wdt_disable() — included in <avr/wdt.h> if needed */

    /* Timer0: 1 ms system tick */
    timer0_ms_init();

    /* Hardware UART for debug */
    uart_init();

    /* Enable global interrupts (needed for Timer0 ISR) */
    sei();

    uart_puts("\r\n== Vehicle Safety System v1.0 ==\r\n");
    uart_puts("Initialising subsystems...\r\n");

    /* ADC (shared by MQ-3 and LDR) */
    mq3_init();      /* Calls adc_init_shared() internally */
    ldr_init();      /* Uses already-initialised ADC        */

    /* I2C devices (TWI init inside mpu6050_init) */
    MPU6050_Status_t imu_status = mpu6050_init();
    if (imu_status != MPU6050_OK) {
        uart_puts("WARN: MPU6050 init failed\r\n");
    }

    /* LCD (uses same I2C bus) */
    lcd_init();
    lcd_show("Vehicle Safety", "System v1.0");
    _delay_ms(2000);

    /* Remaining sensors */
    dht11_init();
    ir_sensor_init();

    /* Collision module (wraps ultrasonic_init) */
    collision_init();

    /* Actuators and LEDs */
    alerts_init();

    /* GPS */
    gps_init();

    /* GSM */
    gsm_init();

    uart_puts("All subsystems ready.\r\n");
    lcd_show("Systems OK", "Warming up MQ3..");
    _delay_ms(1000);
}

/* ════════════════════════════════════════════════════════════════════════════
 *  MAIN LOOP
 * ════════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    /* ── Initialise all subsystems ──────────────────────────────────────── */
    system_init();

    /* ── Local state variables ──────────────────────────────────────────── */
    CollisionData_t  collision_data;
    MPU6050_Data_t   imu_data;
    DHT11_Data_t     dht_data;
    GPS_Data_t       gps_data;
    DrowsyState_t    drowsy = {0.0f, 0, 0};

    uint8_t  alert_flags   = 0;
    AlertLevel_t alert_lvl = ALERT_NONE;

    /* Loop counter for round-robin scheduling of slow sensors */
    uint32_t loop_count    = 0;

    /* Crash SMS rate-limiter: don't send more than 1 SMS per 60 s */
    uint32_t last_sms_ms   = 0;
    uint8_t  crash_latched = 0;  /* Latched until manual reset */

    /* ── GPS data cache (read every 10 loops) ───────────────────────────── */
    float gps_lat = 0.0f, gps_lon = 0.0f, gps_speed = 0.0f;
    uint8_t gps_fix = 0;

    /* ── Main processing loop ───────────────────────────────────────────── */
    while (1) {
        alert_flags = 0;
        alert_lvl   = ALERT_NONE;
        loop_count++;

        /* ════════════════════════════════════════════════════════════════
         *  1. COLLISION AVOIDANCE (every loop)
         * ════════════════════════════════════════════════════════════════ */
        CollisionState_t col_state = collision_update(&collision_data);

        if (col_state == COL_STATE_CRITICAL) {
            /* Critical: engage brake assist for front/rear proximity */
            alert_flags |= ALERT_FLAG_COLLISION;
            alert_lvl    = ALERT_CRITICAL;

            if ((collision_data.direction_flags & COL_DIR_FRONT) ||
                (collision_data.direction_flags & COL_DIR_REAR)) {
                brake_relay_engage();
            }
            display_collision_info(&collision_data);

        } else if (col_state == COL_STATE_WARNING) {
            alert_flags |= ALERT_FLAG_COLLISION;
            if (alert_lvl < ALERT_WARNING) {
                alert_lvl = ALERT_WARNING;
            }
            brake_relay_release();
            display_collision_info(&collision_data);

        } else {
            brake_relay_release();
        }

        /* ════════════════════════════════════════════════════════════════
         *  2. CRASH / ROLLOVER DETECTION (every loop – IMU is fast)
         * ════════════════════════════════════════════════════════════════ */
        if (mpu6050_read(&imu_data) == MPU6050_OK) {

            if ((mpu6050_crash_detected(&imu_data) ||
                 mpu6050_rollover_detected(&imu_data)) && !crash_latched) {

                crash_latched = 1;
                alert_flags  |= ALERT_FLAG_CRASH;
                alert_lvl     = ALERT_CRITICAL;

                uart_puts("CRASH DETECTED! accel=");
                /* Minimal float print: integer part only for debug */
                uart_putchar((char)('0' + (int)imu_data.accel_magnitude_g));
                uart_puts("g\r\n");

                /* Throttle SMS: max one per 60 s */
                if ((g_system_ms - last_sms_ms) > 60000UL || last_sms_ms == 0) {
                    last_sms_ms = g_system_ms;
                    const char *reason = mpu6050_rollover_detected(&imu_data)
                                         ? "ROLLOVER" : "CRASH";
                    handle_emergency(reason, gps_lat, gps_lon, 0);
                }
            }
        }

        /* ════════════════════════════════════════════════════════════════
         *  3. ALCOHOL DETECTION (check every 5 loops; requires warm-up)
         * ════════════════════════════════════════════════════════════════ */
        if ((loop_count % 5U) == 0U) {
            if (mq3_is_warmed_up()) {
                MQ3_Status_t alc = mq3_get_status();

                if (alc == MQ3_DANGER) {
                    alert_flags |= ALERT_FLAG_ALCOHOL;
                    alert_lvl    = ALERT_CRITICAL;

                    /* Cut engine and send SMS if not already sent */
                    if ((g_system_ms - last_sms_ms) > 60000UL || last_sms_ms == 0) {
                        last_sms_ms = g_system_ms;
                        handle_emergency("ALCOHOL DET", gps_lat, gps_lon, 1);
                    } else {
                        engine_relay_cut();
                    }
                    lcd_show("ALCOHOL DANGER!", "Engine OFF");

                } else if (alc == MQ3_WARN) {
                    alert_flags |= ALERT_FLAG_ALCOHOL;
                    if (alert_lvl < ALERT_WARNING) { alert_lvl = ALERT_WARNING; }
                    lcd_show("Alcohol Warning", "Slow down!");

                } else {
                    /* Clean – restore engine if it was cut due to alcohol
                     * (safety: only restore if below danger for >10 s;
                     *  here simplified to immediate restore on CLEAN) */
                    engine_relay_restore();
                }
            } else {
                /* Still warming up */
                lcd_show("MQ3 Warming Up..", "Please wait...");
            }
        }

        /* ════════════════════════════════════════════════════════════════
         *  4. LANE DEPARTURE WARNING (every loop)
         * ════════════════════════════════════════════════════════════════ */
        LaneStatus_t lane = ir_get_lane_status();

        if (lane != LANE_OK && gps_speed > GPS_MIN_SPEED_KMH) {
            /* Only warn if vehicle is moving */
            alert_flags |= ALERT_FLAG_LANE;
            if (alert_lvl < ALERT_WARNING) { alert_lvl = ALERT_WARNING; }

            if (col_state == COL_STATE_CLEAR) {
                /* Only update LCD if no collision alert displayed */
                const char *dir_str = (lane == LANE_DEV_LEFT)  ? "LANE: Drift LEFT" :
                                      (lane == LANE_DEV_RIGHT) ? "LANE: Drift RGHT" :
                                                                  "LANE: Severe!   ";
                lcd_show("Lane Departure!", dir_str);
            }
        }

        /* ════════════════════════════════════════════════════════════════
         *  5. CABIN TEMPERATURE / OVERHEAT (every 10 loops; DHT11 is slow)
         * ════════════════════════════════════════════════════════════════ */
        if ((loop_count % 10U) == 0U) {
            if (dht11_read(&dht_data) == DHT11_OK) {

                if (dht_data.temperature_c >= DHT11_TEMP_DANGER_C) {
                    alert_flags |= ALERT_FLAG_OVERHEAT;
                    alert_lvl    = ALERT_CRITICAL;
                    lcd_show("FIRE / OVERHEAT!", "EVACUATE NOW!!");

                    uart_puts("OVERHEAT: ");
                    uart_putchar((char)('0' + dht_data.temperature_c / 10));
                    uart_putchar((char)('0' + dht_data.temperature_c % 10));
                    uart_puts("C\r\n");

                } else if (dht_data.temperature_c >= DHT11_TEMP_WARN_C) {
                    alert_flags |= ALERT_FLAG_OVERHEAT;
                    if (alert_lvl < ALERT_WARNING) { alert_lvl = ALERT_WARNING; }

                    if (col_state == COL_STATE_CLEAR) {
                        lcd_show("Cabin Temp High!", "Check ventilat.");
                    }
                }
            }
        }

        /* ════════════════════════════════════════════════════════════════
         *  6. AUTO HEADLIGHTS (every 20 loops)
         * ════════════════════════════════════════════════════════════════ */
        if ((loop_count % 20U) == 0U) {
            ldr_update_headlights();
        }

        /* ════════════════════════════════════════════════════════════════
         *  7. GPS READ + DROWSINESS CHECK (every 10 loops)
         * ════════════════════════════════════════════════════════════════ */
        if ((loop_count % 10U) == 0U) {
            if (gps_read(&gps_data) == GPS_OK) {
                gps_lat   = gps_data.latitude;
                gps_lon   = gps_data.longitude;
                gps_speed = gps_data.speed_kmh;
                gps_fix   = 1;

                /* Drowsiness heuristic: speed stable (no course change) for >30s */
                uint32_t now = g_system_ms;
                if (gps_speed > DROWSY_MIN_DRIVE_SPEED_KMH) {
                    float speed_delta = gps_speed - drowsy.last_speed_kmh;
                    if (speed_delta < 0.0f) speed_delta = -speed_delta;

                    if ((now - drowsy.last_check_ms) >= DROWSY_CHECK_INTERVAL_MS) {
                        drowsy.last_check_ms = now;
                        drowsy.last_speed_kmh = gps_speed;

                        if (speed_delta < DROWSY_SPEED_STABLE_KMH) {
                            drowsy.alert_count++;
                            if (drowsy.alert_count >= 2U) {
                                /* Two consecutive 30-second windows with stable speed */
                                alert_flags |= ALERT_FLAG_DROWSY;
                                if (alert_lvl < ALERT_WARNING) {
                                    alert_lvl = ALERT_WARNING;
                                }
                                lcd_show("DROWSY ALERT!", "Take a break!   ");
                                uart_puts("Drowsiness alert triggered\r\n");
                            }
                        } else {
                            /* Speed changed – driver is active */
                            drowsy.alert_count = 0;
                        }
                    }
                } else {
                    /* Vehicle stopped – reset drowsiness counter */
                    drowsy.alert_count    = 0;
                    drowsy.last_check_ms  = g_system_ms;
                    drowsy.last_speed_kmh = gps_speed;
                }
            } else {
                gps_fix = 0;
            }
        }

        /* ════════════════════════════════════════════════════════════════
         *  8. NORMAL STATUS DISPLAY (when no alert is active)
         * ════════════════════════════════════════════════════════════════ */
        if (alert_flags == 0) {
            display_normal_status(gps_speed, dht_data.temperature_c);
        }

        /* ════════════════════════════════════════════════════════════════
         *  9. GLOBAL ALERT OUTPUT (LEDs + Buzzer)
         * ════════════════════════════════════════════════════════════════ */
        alerts_update(alert_flags, alert_lvl);

        /* ════════════════════════════════════════════════════════════════
         *  10. DEBUG SERIAL OUTPUT (every 20 loops)
         * ════════════════════════════════════════════════════════════════ */
        if ((loop_count % 20U) == 0U) {
            uart_puts("LOOP|F:");
            uart_putchar((char)('0' + (collision_data.dist_cm[US_FRONT] > 999 ?
                                       0 : collision_data.dist_cm[US_FRONT] / 100)));
            uart_puts("xx|MQ:");
            uart_putchar((char)('0' + (uint8_t)mq3_get_status()));
            uart_puts("|T:");
            uart_putchar((char)('0' + dht_data.temperature_c / 10));
            uart_putchar((char)('0' + dht_data.temperature_c % 10));
            uart_puts("C|GPS:");
            uart_putchar(gps_fix ? 'Y' : 'N');
            uart_puts("\r\n");
        }

        /* ════════════════════════════════════════════════════════════════
         *  Small inter-loop delay to avoid hammering I2C/sensors
         * ════════════════════════════════════════════════════════════════ */
        _delay_ms(50);

    }  /* while(1) */

    /* Never reached */
    return 0;
}
