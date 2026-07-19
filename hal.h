#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_PORT_A 0U
#define GPIO_PORT_B 1U
#define GPIO_PORT_C 2U
#define GPIO_PORT_D 3U
#define GPIO_MAKE_PIN(port, pin) ((((uint32_t)(port)) << 8) | ((uint32_t)(pin) & 0xFFU))
#define GPIO_PORT_OF(pin) (((uint32_t)(pin) >> 8) & 0xFFU)
#define GPIO_NUMBER_OF(pin) ((uint32_t)(pin) & 0xFFU)
#define GPIO_PIN_0 0U
#define GPIO_PIN_1 1U
#define GPIO_PIN_2 2U
#define GPIO_PIN_3 3U
#define GPIO_PIN_4 4U
#define GPIO_PIN_5 5U
#define GPIO_PIN_6 6U
#define GPIO_PIN_7 7U
#define GPIO_PIN_8 8U
#define GPIO_PIN_9 9U
#define GPIO_PIN_10 10U
#define GPIO_PIN_11 11U
#define GPIO_PIN_12 12U
#define GPIO_PIN_13 13U
#define GPIO_PIN_14 14U
#define GPIO_PIN_15 15U
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_OUTPUT 1U
#define GPIO_MODE_AF 2U
#define GPIO_MODE_ANALOG 3U
#define GPIO_SPEED_LOW 0U
#define GPIO_SPEED_MEDIUM 1U
#define GPIO_SPEED_HIGH 2U
#define GPIO_SPEED_VERY_HIGH 3U

#define TRAFFIC_NORTH_RED GPIO_MAKE_PIN(GPIO_PORT_A, 0U)
#define TRAFFIC_NORTH_YELLOW GPIO_MAKE_PIN(GPIO_PORT_A, 1U)
#define TRAFFIC_NORTH_GREEN GPIO_MAKE_PIN(GPIO_PORT_A, 2U)
#define TRAFFIC_SOUTH_RED GPIO_MAKE_PIN(GPIO_PORT_A, 3U)
#define TRAFFIC_SOUTH_YELLOW GPIO_MAKE_PIN(GPIO_PORT_A, 4U)
#define TRAFFIC_SOUTH_GREEN GPIO_MAKE_PIN(GPIO_PORT_A, 5U)
#define TRAFFIC_EAST_RED GPIO_MAKE_PIN(GPIO_PORT_A, 6U)
#define TRAFFIC_EAST_YELLOW GPIO_MAKE_PIN(GPIO_PORT_A, 7U)
#define TRAFFIC_EAST_GREEN GPIO_MAKE_PIN(GPIO_PORT_A, 8U)
#define TRAFFIC_WEST_RED GPIO_MAKE_PIN(GPIO_PORT_A, 9U)
#define TRAFFIC_WEST_YELLOW GPIO_MAKE_PIN(GPIO_PORT_A, 10U)
#define TRAFFIC_WEST_GREEN GPIO_MAKE_PIN(GPIO_PORT_A, 11U)
#define SENSOR_IR_NORTH GPIO_MAKE_PIN(GPIO_PORT_B, 0U)
#define SENSOR_IR_SOUTH GPIO_MAKE_PIN(GPIO_PORT_B, 1U)
#define SENSOR_IR_EAST GPIO_MAKE_PIN(GPIO_PORT_B, 2U)
#define SENSOR_IR_WEST GPIO_MAKE_PIN(GPIO_PORT_B, 3U)
#define SENSOR_US_TRIG_NORTH GPIO_MAKE_PIN(GPIO_PORT_B, 4U)
#define SENSOR_US_ECHO_NORTH GPIO_MAKE_PIN(GPIO_PORT_B, 5U)
#define SENSOR_US_TRIG_SOUTH GPIO_MAKE_PIN(GPIO_PORT_B, 6U)
#define SENSOR_US_ECHO_SOUTH GPIO_MAKE_PIN(GPIO_PORT_B, 7U)
#define SENSOR_IND_NORTH GPIO_MAKE_PIN(GPIO_PORT_B, 8U)
#define SENSOR_IND_SOUTH GPIO_MAKE_PIN(GPIO_PORT_B, 9U)
#define SENSOR_IND_EAST GPIO_MAKE_PIN(GPIO_PORT_B, 10U)
#define SENSOR_IND_WEST GPIO_MAKE_PIN(GPIO_PORT_B, 11U)

#define UART_BAUD_9600 9600U
#define UART_BAUD_115200 115200U
#define UART_BAUD_921600 921600U
#define UART_PARITY_NONE 0U
#define UART_PARITY_EVEN 1U
#define UART_PARITY_ODD 2U
#define UART_STOP_1 0U
#define UART_STOP_2 1U
#define ADC_CHANNEL_0 0U
#define ADC_CHANNEL_1 1U
#define ADC_CHANNEL_2 2U
#define ADC_CHANNEL_3 3U
#define ADC_RESOLUTION_12B 12U
#define ADC_RESOLUTION_16B 16U
#define PWM_CHANNEL_1 0U
#define PWM_CHANNEL_2 1U
#define PWM_CHANNEL_3 2U
#define PWM_CHANNEL_4 3U
#define PWM_FREQ_1KHZ 1000U
#define PWM_FREQ_10KHZ 10000U

int hal_gpio_init(uint32_t pin, uint8_t mode, uint8_t speed);
int hal_gpio_write(uint32_t pin, bool value);
bool hal_gpio_read(uint32_t pin);
int hal_gpio_toggle(uint32_t pin);
int hal_timer_init(uint32_t period_ms, void (*callback)(void));
int hal_timer_start(void);
int hal_timer_stop(void);
uint32_t hal_get_tick_ms(void);
void hal_delay_ms(uint32_t ms);
int hal_uart_init(uint8_t uart_id, uint32_t baud, uint8_t parity, uint8_t stop_bits);
int hal_uart_send(uint8_t uart_id, const uint8_t *data, uint16_t len);
int hal_uart_receive(uint8_t uart_id, uint8_t *data, uint16_t len, uint32_t timeout_ms);
int hal_uart_printf(uint8_t uart_id, const char *format, ...);
int hal_adc_init(uint8_t channel, uint8_t resolution);
int hal_adc_read(uint8_t channel, uint16_t *value);
float hal_adc_read_voltage(uint8_t channel);
int hal_pwm_init(uint8_t channel, uint32_t freq_hz);
int hal_pwm_set_duty(uint8_t channel, uint8_t duty_percent);
int hal_pwm_start(uint8_t channel);
int hal_pwm_stop(uint8_t channel);
int hal_wdg_init(uint32_t timeout_ms);
int hal_wdg_refresh(void);
int hal_irq_enable(uint8_t irq_num);
int hal_irq_disable(uint8_t irq_num);
int hal_irq_set_priority(uint8_t irq_num, uint8_t priority);
int hal_power_set_mode(uint8_t mode);
int hal_power_get_voltage(float *voltage);
int hal_power_get_current(float *current);
uint32_t hal_crc32(const uint8_t *data, uint32_t len);
uint16_t hal_crc16(const uint8_t *data, uint32_t len);
int hal_flash_erase(uint32_t addr, uint32_t size);
int hal_flash_write(uint32_t addr, const uint8_t *data, uint32_t len);
int hal_flash_read(uint32_t addr, uint8_t *data, uint32_t len);
void hal_system_reset(void);
void hal_enter_bootloader(void);
uint32_t hal_get_unique_id(void);

#ifdef __cplusplus
}
#endif
#endif