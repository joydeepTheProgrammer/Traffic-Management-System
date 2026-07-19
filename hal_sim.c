/**
 * @file hal_sim.c
 * @brief Hardware Abstraction Layer - POSIX Simulation
 * @version 2.0
 * 
 * Simulates GPIO, UART, ADC, PWM for testing on Linux/PC
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <stdarg.h>
#include "hal.h"

/* ============================================================================
 * GPIO SIMULATION
 * ============================================================================ */
static bool gpio_states[4][16] = {{false}};

int hal_gpio_init(uint32_t pin, uint8_t mode, uint8_t speed) {
    (void)mode;
    (void)speed;
    uint8_t port = GPIO_PORT_OF(pin);
    uint8_t pin_num = GPIO_NUMBER_OF(pin);
    if (port >= 4 || pin_num >= 16) return -1;
    gpio_states[port][pin_num] = false;
    return 0;
}

int hal_gpio_write(uint32_t pin, bool value) {
    uint8_t port = GPIO_PORT_OF(pin);
    uint8_t pin_num = GPIO_NUMBER_OF(pin);
    if (port >= 4 || pin_num >= 16) return -1;
    gpio_states[port][pin_num] = value;
    return 0;
}

bool hal_gpio_read(uint32_t pin) {
    uint8_t port = GPIO_PORT_OF(pin);
    uint8_t pin_num = GPIO_NUMBER_OF(pin);
    if (port >= 4 || pin_num >= 16) return false;
    return gpio_states[port][pin_num];
}

int hal_gpio_toggle(uint32_t pin) {
    uint8_t port = GPIO_PORT_OF(pin);
    uint8_t pin_num = GPIO_NUMBER_OF(pin);
    if (port >= 4 || pin_num >= 16) return -1;
    gpio_states[port][pin_num] = !gpio_states[port][pin_num];
    return 0;
}

/* ============================================================================
 * TIMER SIMULATION
 * ============================================================================ */
static struct timeval timer_start;
static void (*timer_callback)(void) = NULL;
static uint32_t timer_period_ms = 0;
static uint32_t timer_last_tick = 0;

int hal_timer_init(uint32_t period_ms, void (*callback)(void)) {
    timer_period_ms = period_ms;
    timer_callback = callback;
    gettimeofday(&timer_start, NULL);
    timer_last_tick = hal_get_tick_ms();
    return 0;
}

int hal_timer_start(void) {
    return 0;
}

int hal_timer_stop(void) {
    return 0;
}

uint32_t hal_get_tick_ms(void) {
    struct timeval now;
    gettimeofday(&now, NULL);
    if (timer_start.tv_sec == 0 && timer_start.tv_usec == 0) {
        timer_start = now;
    }
    return (uint32_t)((now.tv_sec - timer_start.tv_sec) * 1000 +
                      (now.tv_usec - timer_start.tv_usec) / 1000);
}

void hal_delay_ms(uint32_t ms) {
    usleep(ms * 1000);
}

/* ============================================================================
 * UART SIMULATION
 * ============================================================================ */
static struct {
    uint32_t baud;
    uint8_t rx_buffer[1024];
    uint16_t rx_head;
    uint16_t rx_tail;
} uart_channels[4] = {{0}};

int hal_uart_init(uint8_t uart_id, uint32_t baud, uint8_t parity, uint8_t stop_bits) {
    (void)parity;
    (void)stop_bits;
    if (uart_id >= 4) return -1;
    uart_channels[uart_id].baud = baud;
    uart_channels[uart_id].rx_head = 0;
    uart_channels[uart_id].rx_tail = 0;
    return 0;
}

int hal_uart_send(uint8_t uart_id, const uint8_t *data, uint16_t len) {
    if (uart_id >= 4) return -1;
    (void)fwrite(data, 1, len, stdout);
    fflush(stdout);
    return len;
}

int hal_uart_receive(uint8_t uart_id, uint8_t *data, uint16_t len, uint32_t timeout_ms) {
    (void)uart_id;
    (void)timeout_ms;
    /* Return 0 - no data in simulation (would be interrupt-driven on real HW) */
    (void)data;
    (void)len;
    return 0;
}

int hal_uart_printf(uint8_t uart_id, const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len < 0) return -1;
    if (len >= (int)sizeof(buffer)) len = (int)sizeof(buffer) - 1;
    return hal_uart_send(uart_id, (const uint8_t*)buffer, (uint16_t)len);
}

/* ============================================================================
 * ADC SIMULATION
 * ============================================================================ */
static float adc_values[4] = {3.3f, 3.3f, 3.3f, 3.3f};

int hal_adc_init(uint8_t channel, uint8_t resolution) {
    (void)resolution;
    if (channel >= 4) return -1;
    adc_values[channel] = 3.3f;
    return 0;
}

int hal_adc_read(uint8_t channel, uint16_t *value) {
    if (channel >= 4 || value == NULL) return -1;
    /* Simulate some noise */
    float noise = ((float)(rand() % 100) - 50.0f) / 1000.0f;
    float v = adc_values[channel] + noise;
    if (v < 0) v = 0;
    if (v > 3.3f) v = 3.3f;
    *value = (uint16_t)((v / 3.3f) * 4095.0f);
    return 0;
}

float hal_adc_read_voltage(uint8_t channel) {
    uint16_t raw;
    if (hal_adc_read(channel, &raw) != 0) return 0.0f;
    return (raw / 4095.0f) * 3.3f;
}

/* ============================================================================
 * PWM SIMULATION
 * ============================================================================ */
static struct {
    uint32_t freq;
    uint8_t duty;
    bool running;
} pwm_channels[4] = {{0}};

int hal_pwm_init(uint8_t channel, uint32_t freq_hz) {
    if (channel >= 4) return -1;
    pwm_channels[channel].freq = freq_hz;
    pwm_channels[channel].duty = 0;
    pwm_channels[channel].running = false;
    return 0;
}

int hal_pwm_set_duty(uint8_t channel, uint8_t duty_percent) {
    if (channel >= 4) return -1;
    pwm_channels[channel].duty = duty_percent;
    return 0;
}

int hal_pwm_start(uint8_t channel) {
    if (channel >= 4) return -1;
    pwm_channels[channel].running = true;
    return 0;
}

int hal_pwm_stop(uint8_t channel) {
    if (channel >= 4) return -1;
    pwm_channels[channel].running = false;
    return 0;
}

/* ============================================================================
 * WATCHDOG SIMULATION
 * ============================================================================ */
static uint32_t wdg_timeout = 1000;
static uint32_t wdg_last_refresh = 0;

int hal_wdg_init(uint32_t timeout_ms) {
    wdg_timeout = timeout_ms;
    wdg_last_refresh = hal_get_tick_ms();
    return 0;
}

int hal_wdg_refresh(void) {
    wdg_last_refresh = hal_get_tick_ms();
    return 0;
}

/* ============================================================================
 * INTERRUPT SIMULATION
 * ============================================================================ */
int hal_irq_enable(uint8_t irq_num) {
    (void)irq_num;
    return 0;
}

int hal_irq_disable(uint8_t irq_num) {
    (void)irq_num;
    return 0;
}

int hal_irq_set_priority(uint8_t irq_num, uint8_t priority) {
    (void)irq_num;
    (void)priority;
    return 0;
}

/* ============================================================================
 * POWER MANAGEMENT SIMULATION
 * ============================================================================ */
int hal_power_set_mode(uint8_t mode) {
    (void)mode;
    return 0;
}

int hal_power_get_voltage(float *voltage) {
    if (voltage == NULL) return -1;
    *voltage = 12.0f + ((float)(rand() % 10) - 5.0f) / 10.0f;
    return 0;
}

int hal_power_get_current(float *current) {
    if (current == NULL) return -1;
    *current = 0.5f + ((float)(rand() % 10)) / 100.0f;
    return 0;
}

/* ============================================================================
 * CRC SIMULATION
 * ============================================================================ */
uint32_t hal_crc32(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

uint16_t hal_crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc << 1) ^ (0x1021 & ((crc >> 15) ? 0xFFFF : 0));
        }
    }
    return crc;
}

/* ============================================================================
 * FLASH SIMULATION
 * ============================================================================ */
static uint8_t flash_memory[65536] = {0};

int hal_flash_erase(uint32_t addr, uint32_t size) {
    if (addr + size > sizeof(flash_memory)) return -1;
    memset(&flash_memory[addr], 0xFF, size);
    return 0;
}

int hal_flash_write(uint32_t addr, const uint8_t *data, uint32_t len) {
    if (addr + len > sizeof(flash_memory) || data == NULL) return -1;
    memcpy(&flash_memory[addr], data, len);
    return 0;
}

int hal_flash_read(uint32_t addr, uint8_t *data, uint32_t len) {
    if (addr + len > sizeof(flash_memory) || data == NULL) return -1;
    memcpy(data, &flash_memory[addr], len);
    return 0;
}

/* ============================================================================
 * SYSTEM SIMULATION
 * ============================================================================ */
void hal_system_reset(void) {
    printf("\n[HAL] SYSTEM RESET TRIGGERED\n");
    exit(0);
}

void hal_enter_bootloader(void) {
    printf("\n[HAL] ENTERING BOOTLOADER\n");
}

uint32_t hal_get_unique_id(void) {
    return 0xDEADBEEF;
}
