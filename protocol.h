/**
 * @file protocol.h
 * @brief Traffic Management Communication Protocol
 * @version 2.0
 * 
 * Protocol: Binary frame-based with CRC32
 * Frame Structure:
 *   [SYNC:2][LEN:2][CMD:1][DATA:N][CRC:4] = 9+N bytes
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_SYNC_BYTE_1       0x55
#define PROTO_SYNC_BYTE_2       0xAA
#define PROTO_MAX_PAYLOAD       256
#define PROTO_HEADER_SIZE       5
#define PROTO_CRC_SIZE          4
#define PROTO_MAX_FRAME_SIZE    (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE)

/* Command IDs */
typedef enum {
    /* System Commands */
    CMD_HEARTBEAT = 0x01,
    CMD_STATUS_REQ = 0x02,
    CMD_STATUS_RESP = 0x03,
    CMD_RESET = 0x04,
    CMD_SET_MODE = 0x05,

    /* Traffic Control */
    CMD_SET_LIGHT = 0x10,
    CMD_GET_LIGHT = 0x11,
    CMD_SET_TIMING = 0x12,
    CMD_GET_TIMING = 0x13,
    CMD_FORCE_PHASE = 0x14,

    /* Sensor Commands */
    CMD_SENSOR_READ = 0x20,
    CMD_SENSOR_CONFIG = 0x21,
    CMD_SENSOR_CALIBRATE = 0x22,

    /* Emergency */
    CMD_EMERGENCY_TRIGGER = 0x30,
    CMD_EMERGENCY_CLEAR = 0x31,

    /* Pedestrian */
    CMD_PEDESTRIAN_REQUEST = 0x40,
    CMD_PEDESTRIAN_CLEAR = 0x41,

    /* Data Logging */
    CMD_LOG_READ = 0x50,
    CMD_LOG_CLEAR = 0x51,
    CMD_LOG_CONFIG = 0x52,

    /* Firmware Update */
    CMD_FW_UPDATE_START = 0x60,
    CMD_FW_UPDATE_DATA = 0x61,
    CMD_FW_UPDATE_VERIFY = 0x62,
    CMD_FW_UPDATE_COMMIT = 0x63,

    /* Error Response */
    CMD_ERROR = 0xFF
} ProtocolCommand;

/* Response Codes */
typedef enum {
    RESP_OK = 0x00,
    RESP_INVALID_CMD = 0x01,
    RESP_INVALID_PARAM = 0x02,
    RESP_CRC_ERROR = 0x03,
    RESP_TIMEOUT = 0x04,
    RESP_BUSY = 0x05,
    RESP_NOT_SUPPORTED = 0x06,
    RESP_SYSTEM_FAULT = 0x07,
    RESP_BUFFER_OVERFLOW = 0x08
} ResponseCode;

/* Frame Structure */
typedef struct __attribute__((packed)) {
    uint8_t sync1;
    uint8_t sync2;
    uint16_t length;
    uint8_t cmd;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint32_t crc;
} ProtocolFrame;

/* Status Response Payload */
typedef struct __attribute__((packed)) {
    uint8_t mode;
    uint8_t fault_status;
    uint16_t queue_count;
    uint32_t vehicles_processed;
    uint32_t pedestrians_processed;
    uint8_t emergency_count;
    uint8_t direction_states[4];
    uint16_t direction_counts[4];
    uint16_t direction_pedestrians[4];
    uint32_t uptime_seconds;
    float system_voltage;
    float system_current;
    int8_t temperature;
} StatusPayload;

/* Light Control Payload */
typedef struct __attribute__((packed)) {
    uint8_t direction;
    uint8_t state;
    uint16_t duration;
} LightControlPayload;

/* Timing Config Payload */
typedef struct __attribute__((packed)) {
    uint8_t direction;
    uint16_t min_green;
    uint16_t max_green;
    uint16_t yellow_time;
    uint16_t all_red_time;
} TimingConfigPayload;

/* Sensor Read Payload */
typedef struct __attribute__((packed)) {
    uint8_t direction;
    uint8_t sensor_type;
    float value;
    uint8_t confidence;
} SensorReadPayload;

/* Emergency Payload */
typedef struct __attribute__((packed)) {
    uint8_t direction;
    uint32_t vehicle_id;
} EmergencyPayload;

/* Log Entry Payload */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    uint8_t severity;
    uint8_t module_id;
    char message[64];
} LogEntryPayload;

/* Firmware Update Payload */
typedef struct __attribute__((packed)) {
    uint32_t offset;
    uint16_t data_len;
    uint8_t data[240];
    uint32_t chunk_crc;
} FwUpdatePayload;

/* ============================================================================
 * API
 * ============================================================================ */
int protocol_init(void);
int protocol_encode_frame(ProtocolFrame *frame, uint8_t cmd, const uint8_t *payload, uint16_t len);
int protocol_decode_frame(const uint8_t *raw, uint16_t raw_len, ProtocolFrame *frame);
bool protocol_validate_frame(const ProtocolFrame *frame);
uint32_t protocol_calculate_crc(const uint8_t *data, uint16_t len);
int protocol_send_frame(uint8_t uart_id, const ProtocolFrame *frame);
int protocol_receive_frame(uint8_t uart_id, ProtocolFrame *frame, uint32_t timeout_ms);

/* Response helpers */
int protocol_send_response(uint8_t uart_id, uint8_t cmd, ResponseCode code, const uint8_t *data, uint16_t len);
int protocol_send_error(uint8_t uart_id, ResponseCode code);
int protocol_send_status(uint8_t uart_id);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
