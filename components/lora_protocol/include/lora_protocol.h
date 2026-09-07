#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_PROTOCOL_VERSION 2U
#define LORA_PROTOCOL_ZONE_COUNT 64U

/* Every uplink packet carries the same header so the receiver and the UI get
 * fresh config and timing telemetry no matter which packet arrives. */
#define LORA_PROTOCOL_UPLINK_HEADER_SIZE 50U
#define LORA_PROTOCOL_WAKE_SIZE 52U
#define LORA_PROTOCOL_STATUS_SIZE 52U
#define LORA_PROTOCOL_CONFIG_ACK_SIZE 53U
#define LORA_PROTOCOL_SLEEPING_SIZE 56U
#define LORA_PROTOCOL_FRAME_SIZE 188U
#define LORA_PROTOCOL_CONFIG_SET_SIZE 16U

/* SX1262 carries at most 255 bytes in one packet. */
#define LORA_PROTOCOL_MAX_PACKET_SIZE LORA_PROTOCOL_FRAME_SIZE

/* Node-specific transmit offset that keeps B-1 and B-2 from colliding. */
#define LORA_PROTOCOL_TX_SLOT_MS 500U

typedef enum {
    LORA_PACKET_WAKE = 1,
    LORA_PACKET_FRAME = 2,
    LORA_PACKET_STATUS = 3,
    LORA_PACKET_SLEEPING = 4,
    LORA_PACKET_CONFIG_SET = 5,
    LORA_PACKET_CONFIG_ACK = 6,
} lora_packet_type_t;

typedef enum {
    LORA_STATUS_BOOT = 1,
    LORA_STATUS_TOF_NOT_FOUND = 2,
    LORA_STATUS_TOF_INIT_FAILED = 3,
    LORA_STATUS_TOF_READ_FAILED = 5,
    LORA_STATUS_TOF_SLEEP_FAILED = 7,
    LORA_STATUS_RADIO_SLEEP_FAILED = 8,
} lora_status_code_t;

typedef enum {
    LORA_ACK_APPLIED_NEXT_CYCLE = 0,
    LORA_ACK_REJECTED_RANGE = 1,
    LORA_ACK_REJECTED_STALE = 2,
} lora_ack_result_t;

/* Runtime settings owned by the UI and pushed down through A. */
typedef struct {
    uint16_t revision;
    uint16_t sleep_s;
    uint16_t wake_s;
    uint8_t measure_count;
} lora_config_t;

/* v1 folded sleep into a single 32-bit uptime, which wrapped at ~49.7 days and
 * could not tell awake time from sleep time. These are separate and 64-bit. */
typedef struct {
    uint32_t awake_elapsed_ms;
    uint32_t last_sleep_ms;
    uint64_t total_awake_ms;
    uint64_t total_sleep_ms;
    uint32_t wake_count;
    /* Battery millivolts, or 0 when the reading is unavailable or implausible.
     * A sealed node that goes quiet is otherwise impossible to tell apart from
     * a flat cell without opening the cartridge. */
    uint16_t vbat_mv;
} lora_telemetry_t;

/* Shared by every uplink encoder. */
typedef struct {
    uint8_t node_id;
    uint32_t sequence;
    uint8_t retry_count;
    lora_config_t config;
    lora_telemetry_t telemetry;
} lora_uplink_t;

typedef struct {
    lora_packet_type_t type;
    uint8_t node_id;
    uint32_t sequence;
    uint8_t retry_count;
    lora_config_t config;
    lora_telemetry_t telemetry;

    uint8_t status_code;       /* STATUS */
    uint8_t ack_result;        /* CONFIG_ACK */
    uint32_t planned_sleep_ms; /* SLEEPING */

    uint64_t valid_mask;       /* FRAME */
    uint16_t distance_mm[LORA_PROTOCOL_ZONE_COUNT];
} lora_packet_t;

/* All encoders return the number of bytes written, or 0 on bad arguments or
 * insufficient capacity. */
size_t lora_protocol_encode_wake(uint8_t *output, size_t capacity,
                                 const lora_uplink_t *uplink);

size_t lora_protocol_encode_frame(uint8_t *output, size_t capacity,
                                  const lora_uplink_t *uplink,
                                  uint64_t valid_mask,
                                  const uint16_t *distance_mm);

size_t lora_protocol_encode_status(uint8_t *output, size_t capacity,
                                   const lora_uplink_t *uplink,
                                   uint8_t status_code);

size_t lora_protocol_encode_sleeping(uint8_t *output, size_t capacity,
                                     const lora_uplink_t *uplink,
                                     uint32_t planned_sleep_ms);

size_t lora_protocol_encode_config_ack(uint8_t *output, size_t capacity,
                                       const lora_uplink_t *uplink,
                                       uint8_t result);

size_t lora_protocol_encode_config_set(uint8_t *output, size_t capacity,
                                       uint8_t target_node_id,
                                       const lora_config_t *config);

/* Returns 0 on success, -1 when the magic, version, length or CRC is wrong. */
int lora_protocol_decode(const uint8_t *data, size_t length,
                         lora_packet_t *packet);

/* True when the values are inside the ranges a node will accept. */
bool lora_protocol_config_is_valid(const lora_config_t *config);

/* Deterministic per-node delay so two nodes waking together do not transmit at
 * the same instant. */
uint32_t lora_protocol_tx_slot_offset_ms(uint8_t node_id);

#ifdef __cplusplus
}
#endif
