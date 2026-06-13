/**
 * @file hal.h
 * @brief Hardware Abstraction Layer - GPIO, Timer, UART, ADC, PWM
 * @version 2.0
 */

#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * GPIO Definitions
 * ============================================================================ */
#define GPIO_PORT_A     0
#define GPIO_PORT_B     1
#define GPIO_PORT_C     2
#define GPIO_PORT_D     3

#define GPIO_PIN_0      (1U << 0)
#define GPIO_PIN_1      (1U << 1)
#define GPIO_PIN_2      (1U << 2)
#define GPIO_PIN_3      (1U << 3)
#define GPIO_PIN_4      (1U << 4)
#define GPIO_PIN_5      (1U << 5)
#define GPIO_PIN_6      (1U << 6)
#define GPIO_PIN_7      (1U << 7)
#define GPIO_PIN_8      (1U << 8)
#define GPIO_PIN_9      (1U << 9)
#define GPIO_PIN_10     (1U << 10)
#define GPIO_PIN_11     (1U << 11)
#define GPIO_PIN_12     (1U << 12)
#define GPIO_PIN_13     (1U << 13)
#define GPIO_PIN_14     (1U << 14)
#define GPIO_PIN_15     (1U << 15)

#define GPIO_MODE_INPUT     0
#define GPIO_MODE_OUTPUT    1
#define GPIO_MODE_AF        2
#define GPIO_MODE_ANALOG    3

#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_HIGH     2
#define GPIO_SPEED_VERY_HIGH 3

/* Traffic Light Pin Mapping (4 directions x 3 colors) */
#define TRAFFIC_NORTH_RED       (GPIO_PORT_A | GPIO_PIN_0)
#define TRAFFIC_NORTH_YELLOW    (GPIO_PORT_A | GPIO_PIN_1)
#define TRAFFIC_NORTH_GREEN     (GPIO_PORT_A | GPIO_PIN_2)
#define TRAFFIC_SOUTH_RED       (GPIO_PORT_A | GPIO_PIN_3)
#define TRAFFIC_SOUTH_YELLOW    (GPIO_PORT_A | GPIO_PIN_4)
#define TRAFFIC_SOUTH_GREEN     (GPIO_PORT_A | GPIO_PIN_5)
#define TRAFFIC_EAST_RED        (GPIO_PORT_A | GPIO_PIN_6)
#define TRAFFIC_EAST_YELLOW     (GPIO_PORT_A | GPIO_PIN_7)
#define TRAFFIC_EAST_GREEN      (GPIO_PORT_A | GPIO_PIN_8)
#define TRAFFIC_WEST_RED        (GPIO_PORT_A | GPIO_PIN_9)
#define TRAFFIC_WEST_YELLOW     (GPIO_PORT_A | GPIO_PIN_10)
#define TRAFFIC_WEST_GREEN      (GPIO_PORT_A | GPIO_PIN_11)

/* Sensor Pin Mapping */
#define SENSOR_IR_NORTH         (GPIO_PORT_B | GPIO_PIN_0)
#define SENSOR_IR_SOUTH         (GPIO_PORT_B | GPIO_PIN_1)
#define SENSOR_IR_EAST          (GPIO_PORT_B | GPIO_PIN_2)
#define SENSOR_IR_WEST          (GPIO_PORT_B | GPIO_PIN_3)
#define SENSOR_US_TRIG_NORTH    (GPIO_PORT_B | GPIO_PIN_4)
#define SENSOR_US_ECHO_NORTH    (GPIO_PORT_B | GPIO_PIN_5)
#define SENSOR_US_TRIG_SOUTH    (GPIO_PORT_B | GPIO_PIN_6)
#define SENSOR_US_ECHO_SOUTH    (GPIO_PORT_B | GPIO_PIN_7)
#define SENSOR_IND_NORTH        (GPIO_PORT_B | GPIO_PIN_8)
#define SENSOR_IND_SOUTH        (GPIO_PORT_B | GPIO_PIN_9)
#define SENSOR_IND_EAST         (GPIO_PORT_B | GPIO_PIN_10)
#define SENSOR_IND_WEST         (GPIO_PORT_B | GPIO_PIN_11)

/* Emergency/Pedestrian Buttons */
#define BTN_EMERGENCY_NORTH     (GPIO_PORT_C | GPIO_PIN_0)
#define BTN_EMERGENCY_SOUTH     (GPIO_PORT_C | GPIO_PIN_1)
#define BTN_EMERGENCY_EAST      (GPIO_PORT_C | GPIO_PIN_2)
#define BTN_EMERGENCY_WEST      (GPIO_PORT_C | GPIO_PIN_3)
#define BTN_PEDESTRIAN_NORTH    (GPIO_PORT_C | GPIO_PIN_4)
#define BTN_PEDESTRIAN_SOUTH    (GPIO_PORT_C | GPIO_PIN_5)
#define BTN_PEDESTRIAN_EAST     (GPIO_PORT_C | GPIO_PIN_6)
#define BTN_PEDESTRIAN_WEST     (GPIO_PORT_C | GPIO_PIN_7)

/* ============================================================================
 * UART Definitions
 * ============================================================================ */
#define UART_BAUD_9600      9600
#define UART_BAUD_115200    115200
#define UART_BAUD_921600    921600

#define UART_PARITY_NONE    0
#define UART_PARITY_EVEN    1
#define UART_PARITY_ODD     2

#define UART_STOP_1         0
#define UART_STOP_2         1

/* ============================================================================
 * ADC Definitions
 * ============================================================================ */
#define ADC_CHANNEL_0       0
#define ADC_CHANNEL_1       1
#define ADC_CHANNEL_2       2
#define ADC_CHANNEL_3       3
#define ADC_RESOLUTION_12B  12
#define ADC_RESOLUTION_16B  16

/* ============================================================================
 * PWM Definitions
 * ============================================================================ */
#define PWM_CHANNEL_1       0
#define PWM_CHANNEL_2       1
#define PWM_CHANNEL_3       2
#define PWM_CHANNEL_4       3
#define PWM_FREQ_1KHZ       1000
#define PWM_FREQ_10KHZ      10000

/* ============================================================================
 * Watchdog Definitions
 * ============================================================================ */
#define WDG_TIMEOUT_MS      1000

/* ============================================================================
 * HAL API
 * ============================================================================ */

/* GPIO */
int hal_gpio_init(uint32_t pin, uint8_t mode, uint8_t speed);
int hal_gpio_write(uint32_t pin, bool value);
bool hal_gpio_read(uint32_t pin);
int hal_gpio_toggle(uint32_t pin);

/* Timer */
int hal_timer_init(uint32_t period_ms, void (*callback)(void));
int hal_timer_start(void);
int hal_timer_stop(void);
uint32_t hal_get_tick_ms(void);
void hal_delay_ms(uint32_t ms);

/* UART */
int hal_uart_init(uint8_t uart_id, uint32_t baud, uint8_t parity, uint8_t stop_bits);
int hal_uart_send(uint8_t uart_id, const uint8_t *data, uint16_t len);
int hal_uart_receive(uint8_t uart_id, uint8_t *data, uint16_t len, uint32_t timeout_ms);
int hal_uart_printf(uint8_t uart_id, const char *format, ...);

/* ADC */
int hal_adc_init(uint8_t channel, uint8_t resolution);
int hal_adc_read(uint8_t channel, uint16_t *value);
float hal_adc_read_voltage(uint8_t channel);

/* PWM */
int hal_pwm_init(uint8_t channel, uint32_t freq_hz);
int hal_pwm_set_duty(uint8_t channel, uint8_t duty_percent);
int hal_pwm_start(uint8_t channel);
int hal_pwm_stop(uint8_t channel);

/* Watchdog */
int hal_wdg_init(uint32_t timeout_ms);
int hal_wdg_refresh(void);

/* Interrupt */
int hal_irq_enable(uint8_t irq_num);
int hal_irq_disable(uint8_t irq_num);
int hal_irq_set_priority(uint8_t irq_num, uint8_t priority);

/* Power Management */
int hal_power_set_mode(uint8_t mode);
int hal_power_get_voltage(float *voltage);
int hal_power_get_current(float *current);

/* CRC */
uint32_t hal_crc32(const uint8_t *data, uint32_t len);
uint16_t hal_crc16(const uint8_t *data, uint32_t len);

/* Flash */
int hal_flash_erase(uint32_t addr, uint32_t size);
int hal_flash_write(uint32_t addr, const uint8_t *data, uint32_t len);
int hal_flash_read(uint32_t addr, uint8_t *data, uint32_t len);

/* System */
void hal_system_reset(void);
void hal_enter_bootloader(void);
uint32_t hal_get_unique_id(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_H */
