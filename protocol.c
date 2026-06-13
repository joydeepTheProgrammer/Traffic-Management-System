/**
 * @file protocol.c
 * @brief Binary Communication Protocol Implementation
 * @version 2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "protocol.h"
#include "hal.h"
#include "traffic_system.h"

/* ============================================================================
 * INTERNAL STATE
 * ============================================================================ */
static uint8_t tx_buffer[PROTO_MAX_FRAME_SIZE];
static uint32_t frame_sequence = 0;

/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */
int protocol_init(void) {
    memset(tx_buffer, 0, sizeof(tx_buffer));
    frame_sequence = 0;
    return 0;
}

/* ============================================================================
 * CRC CALCULATION
 * ============================================================================ */
uint32_t protocol_calculate_crc(const uint8_t *data, uint16_t len) {
    return hal_crc32(data, len);
}

/* ============================================================================
 * FRAME ENCODING
 * ============================================================================ */
int protocol_encode_frame(ProtocolFrame *frame, uint8_t cmd, const uint8_t *payload, uint16_t len) {
    if (frame == NULL || len > PROTO_MAX_PAYLOAD) return -1;

    frame->sync1 = PROTO_SYNC_BYTE_1;
    frame->sync2 = PROTO_SYNC_BYTE_2;
    frame->length = len;
    frame->cmd = cmd;

    if (payload != NULL && len > 0) {
        memcpy(frame->payload, payload, len);
    }

    /* Calculate CRC over header + payload */
    uint16_t crc_len = PROTO_HEADER_SIZE - 2 + len; /* sync bytes excluded from CRC */
    frame->crc = protocol_calculate_crc((const uint8_t*)&frame->length, crc_len);

    return 0;
}

/* ============================================================================
 * FRAME DECODING
 * ============================================================================ */
int protocol_decode_frame(const uint8_t *raw, uint16_t raw_len, ProtocolFrame *frame) {
    if (raw == NULL || frame == NULL || raw_len < PROTO_HEADER_SIZE + PROTO_CRC_SIZE) {
        return -1;
    }

    /* Verify sync bytes */
    if (raw[0] != PROTO_SYNC_BYTE_1 || raw[1] != PROTO_SYNC_BYTE_2) {
        return -1;
    }

    /* Extract length */
    uint16_t payload_len = (raw[2] << 8) | raw[3];
    if (payload_len > PROTO_MAX_PAYLOAD) {
        return -1;
    }

    /* Verify total length */
    uint16_t expected_len = PROTO_HEADER_SIZE + payload_len + PROTO_CRC_SIZE;
    if (raw_len < expected_len) {
        return -1;
    }

    /* Copy frame */
    frame->sync1 = raw[0];
    frame->sync2 = raw[1];
    frame->length = payload_len;
    frame->cmd = raw[4];

    if (payload_len > 0) {
        memcpy(frame->payload, &raw[5], payload_len);
    }

    /* Extract CRC */
    uint16_t crc_offset = PROTO_HEADER_SIZE + payload_len;
    frame->crc = ((uint32_t)raw[crc_offset] << 24) |
                 ((uint32_t)raw[crc_offset + 1] << 16) |
                 ((uint32_t)raw[crc_offset + 2] << 8) |
                 (uint32_t)raw[crc_offset + 3];

    /* Validate CRC */
    uint16_t crc_len = PROTO_HEADER_SIZE - 2 + payload_len;
    uint32_t calc_crc = protocol_calculate_crc(&raw[2], crc_len);

    if (calc_crc != frame->crc) {
        return -2; /* CRC mismatch */
    }

    return 0;
}

bool protocol_validate_frame(const ProtocolFrame *frame) {
    if (frame == NULL) return false;
    if (frame->sync1 != PROTO_SYNC_BYTE_1 || frame->sync2 != PROTO_SYNC_BYTE_2) {
        return false;
    }
    if (frame->length > PROTO_MAX_PAYLOAD) return false;
    if (frame->cmd == 0x00) return false; /* Reserved */
    return true;
}

/* ============================================================================
 * FRAME TRANSMISSION
 * ============================================================================ */
int protocol_send_frame(uint8_t uart_id, const ProtocolFrame *frame) {
    if (frame == NULL) return -1;

    uint16_t total_len = PROTO_HEADER_SIZE + frame->length + PROTO_CRC_SIZE;

    /* Build raw frame */
    tx_buffer[0] = frame->sync1;
    tx_buffer[1] = frame->sync2;
    tx_buffer[2] = (frame->length >> 8) & 0xFF;
    tx_buffer[3] = frame->length & 0xFF;
    tx_buffer[4] = frame->cmd;

    if (frame->length > 0) {
        memcpy(&tx_buffer[5], frame->payload, frame->length);
    }

    uint16_t crc_offset = PROTO_HEADER_SIZE + frame->length;
    tx_buffer[crc_offset] = (frame->crc >> 24) & 0xFF;
    tx_buffer[crc_offset + 1] = (frame->crc >> 16) & 0xFF;
    tx_buffer[crc_offset + 2] = (frame->crc >> 8) & 0xFF;
    tx_buffer[crc_offset + 3] = frame->crc & 0xFF;

    return hal_uart_send(uart_id, tx_buffer, total_len);
}

int protocol_receive_frame(uint8_t uart_id, ProtocolFrame *frame, uint32_t timeout_ms) {
    (void)uart_id;
    (void)timeout_ms;

    /* In real implementation, this would read from UART ring buffer */
    /* For simulation, return timeout */
    (void)frame;
    return -1;
}

/* ============================================================================
 * RESPONSE HELPERS
 * ============================================================================ */
int protocol_send_response(uint8_t uart_id, uint8_t cmd, ResponseCode code, 
                           const uint8_t *data, uint16_t len) {
    ProtocolFrame frame;
    uint8_t payload[PROTO_MAX_PAYLOAD];

    payload[0] = code;
    uint16_t payload_len = 1;

    if (data != NULL && len > 0) {
        if (len > PROTO_MAX_PAYLOAD - 1) len = PROTO_MAX_PAYLOAD - 1;
        memcpy(&payload[1], data, len);
        payload_len += len;
    }

    if (protocol_encode_frame(&frame, cmd, payload, payload_len) != 0) {
        return -1;
    }

    return protocol_send_frame(uart_id, &frame);
}

int protocol_send_error(uint8_t uart_id, ResponseCode code) {
    return protocol_send_response(uart_id, CMD_ERROR, code, NULL, 0);
}

int protocol_send_status(uint8_t uart_id) {
    StatusPayload status;

    status.mode = (uint8_t)g_system.current_mode;
    status.fault_status = g_system.system_fault ? 1 : 0;
    status.queue_count = (uint16_t)g_system.queue_count;
    status.vehicles_processed = g_system.total_vehicles_processed;
    status.pedestrians_processed = g_system.total_pedestrians_processed;
    status.emergency_count = (uint8_t)g_system.emergency_count;

    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        status.direction_states[d] = (uint8_t)g_system.lanes[d].state;
        status.direction_counts[d] = (uint16_t)g_system.lanes[d].vehicle_count;
        status.direction_pedestrians[d] = (uint16_t)g_system.lanes[d].pedestrian_waiting;
    }

    struct timeval now;
    gettimeofday(&now, NULL);
    status.uptime_seconds = (uint32_t)(now.tv_sec - g_system.start_time.tv_sec);

    float voltage, current;
    hal_power_get_voltage(&voltage);
    hal_power_get_current(&current);
    status.system_voltage = voltage;
    status.system_current = current;
    status.temperature = 25; /* Simulated */

    return protocol_send_response(uart_id, CMD_STATUS_RESP, RESP_OK,
                                  (uint8_t*)&status, sizeof(status));
}
