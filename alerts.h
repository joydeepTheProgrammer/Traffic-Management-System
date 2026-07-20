/**
 * @file alerts.h
 * @brief Alerts and Actuator control header
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Manages:
 *  - Piezo buzzer (PWM tone, beep patterns)
 *  - Engine cut-off relay (active LOW relay module)
 *  - Brake assist relay  (active LOW relay module)
 *  - LED warning indicators
 */

#ifndef ALERTS_H
#define ALERTS_H

#include <stdint.h>
#include <avr/io.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Pin Assignments
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Buzzer: Timer2 OC2B (PD3) – Note: This is PORTD PD3 which doubles as
 * GSM TX. In this system the buzzer uses a separate GPIO toggle (no PWM)
 * because PD3 is used for GSM soft-UART. Buzzer on PB5 instead. */
#define BUZZER_DDR    DDRB
#define BUZZER_PORT   PORTB
#define BUZZER_PIN    PB5

/* Engine cut-off relay (LOW = relay ON = engine disabled) */
#define ENGINE_RELAY_DDR    DDRD
#define ENGINE_RELAY_PORT   PORTD
#define ENGINE_RELAY_PIN    PD0

/* Brake assist relay (LOW = relay ON = brake engaged) */
#define BRAKE_RELAY_DDR     DDRD
#define BRAKE_RELAY_PORT    PORTD
#define BRAKE_RELAY_PIN     PD1

/* LED indicators on PORTB */
#define LED_COLLISION_DDR   DDRB
#define LED_COLLISION_PORT  PORTB
#define LED_COLLISION_PIN   PB6   /**< Red LED – Collision warning  */

#define LED_ALCOHOL_DDR     DDRB
#define LED_ALCOHOL_PORT    PORTB
#define LED_ALCOHOL_PIN     PB7   /**< Yellow LED – Alcohol warning */

/* ── IMPORTANT: PD3=GSM_TX, PD4=GSM_RX, PD5=GPS_TX — all used by soft UART.
 *  Crash/Lane/Temp LEDs CANNOT be placed on PD3-PD5 on the ATmega328P.
 *  Options:
 *    a) Drive via PCF8574 I2C I/O expander (same I2C bus as LCD)
 *    b) Use a shift register on spare SPI pins
 *    c) If GPS is unused, PD5 can host LED_TEMP.
 *  In this firmware the three LEDs are driven via the LCD PCF8574 backpack
 *  GP4/GP5/GP6 outputs — not direct GPIO. The #defines below are
 *  placeholders; actual control is in alerts.c via lcd_i2c expander bits. */

/* Placeholder only — NOT direct GPIO (see note above) */
#define LED_CRASH_EXPANDER_BIT  4U  /**< PCF8574 GP4 – Red flash – Crash    */
#define LED_LANE_EXPANDER_BIT   5U  /**< PCF8574 GP5 – Amber  – Lane depart */
#define LED_TEMP_EXPANDER_BIT   6U  /**< PCF8574 GP6 – Red    – Overheat    */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Alert Priority Levels
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef enum {
    ALERT_NONE     = 0,
    ALERT_INFO,         /**< Single short beep        */
    ALERT_WARNING,      /**< Three short beeps         */
    ALERT_CRITICAL      /**< Continuous rapid beeping  */
} AlertLevel_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Alert type bitmask – allows multiple simultaneous alerts
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ALERT_FLAG_COLLISION  (1U << 0)
#define ALERT_FLAG_ALCOHOL    (1U << 1)
#define ALERT_FLAG_CRASH      (1U << 2)
#define ALERT_FLAG_LANE       (1U << 3)
#define ALERT_FLAG_OVERHEAT   (1U << 4)
#define ALERT_FLAG_DROWSY     (1U << 5)
#define ALERT_FLAG_HEADLIGHT  (1U << 6)

/* ═══════════════════════════════════════════════════════════════════════════
 *  API
 * ═══════════════════════════════════════════════════════════════════════════ */
void alerts_init(void);

/* Buzzer */
void buzzer_beep(AlertLevel_t level);
void buzzer_on(void);
void buzzer_off(void);

/* Relay actuators */
void engine_relay_cut(void);    /**< Disable engine (relay ON)   */
void engine_relay_restore(void);/**< Re-enable engine (relay OFF) */
void brake_relay_engage(void);  /**< Apply brake assist          */
void brake_relay_release(void); /**< Release brake assist        */

/* LED control */
void led_set(uint8_t alert_flags);
void led_clear_all(void);

/* Composite alert handler – call in main loop */
void alerts_update(uint8_t alert_flags, AlertLevel_t level);

#endif /* ALERTS_H */
