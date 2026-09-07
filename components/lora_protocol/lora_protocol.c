#include "lora_protocol.h"

#include <string.h>

#define MAGIC_0 ((uint8_t)'T')
#define MAGIC_1 ((uint8_t)'F')

/* Common uplink header layout. */
#define OFF_MAGIC_0 0U
#define OFF_MAGIC_1 1U
#define OFF_VERSION 2U
#define OFF_TYPE 3U
#define OFF_NODE_ID 4U
#define OFF_ROWS 5U
#define OFF_COLS 6U
#define OFF_STATUS_CODE 7U
#define OFF_SEQUENCE 8U
#define OFF_CONFIG_REVISION 12U
#define OFF_SLEEP_S 14U
#define OFF_WAKE_S 16U
#define OFF_MEASURE_COUNT 18U
#define OFF_RETRY_COUNT 19U
#define OFF_AWAKE_ELAPSED_MS 20U
#define OFF_LAST_SLEEP_MS 24U
#define OFF_TOTAL_AWAKE_MS 28U
#define OFF_TOTAL_SLEEP_MS 36U
#define OFF_WAKE_COUNT 44U
#define OFF_VBAT_MV 48U

/* Type-specific payloads start right after the common header. */
#define OFF_FRAME_VALID_MASK LORA_PROTOCOL_UPLINK_HEADER_SIZE
#define OFF_FRAME_DISTANCES (LORA_PROTOCOL_UPLINK_HEADER_SIZE + 8U)
#define OFF_SLEEPING_PLANNED_MS LORA_PROTOCOL_UPLINK_HEADER_SIZE
#define OFF_ACK_RESULT LORA_PROTOCOL_UPLINK_HEADER_SIZE

/* CONFIG_SET is a downlink and does not carry telemetry. */
#define OFF_SET_REVISION 6U
#define OFF_SET_SLEEP_S 8U
#define OFF_SET_WAKE_S 10U
#define OFF_SET_MEASURE_COUNT 12U

#define CONFIG_MAX_SLEEP_S 3600U
#define CONFIG_MAX_WAKE_S 3600U
#define CONFIG_MAX_MEASURE_COUNT 10U

static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    for (size_t i = 0; i < 4; ++i) {
        destination[i] = (uint8_t)(value >> (8U * i));
    }
}

static void put_u64_le(uint8_t *destination, uint64_t value)
{
    for (size_t i = 0; i < 8; ++i) {
        destination[i] = (uint8_t)(value >> (8U * i));
    }
}

static uint16_t get_u16_le(const uint8_t *source)
{
    return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}

static uint32_t get_u32_le(const uint8_t *source)
{
    uint32_t value = 0;
    for (size_t i = 0; i < 4; ++i) {
        value |= (uint32_t)source[i] << (8U * i);
    }
    return value;
}

static uint64_t get_u64_le(const uint8_t *source)
{
    uint64_t value = 0;
    for (size_t i = 0; i < 8; ++i) {
        value |= (uint64_t)source[i] << (8U * i);
    }
    return value;
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static size_t expected_size(lora_packet_type_t type)
{
    switch (type) {
    case LORA_PACKET_WAKE:
        return LORA_PROTOCOL_WAKE_SIZE;
    case LORA_PACKET_FRAME:
        return LORA_PROTOCOL_FRAME_SIZE;
    case LORA_PACKET_STATUS:
        return LORA_PROTOCOL_STATUS_SIZE;
    case LORA_PACKET_SLEEPING:
        return LORA_PROTOCOL_SLEEPING_SIZE;
    case LORA_PACKET_CONFIG_SET:
        return LORA_PROTOCOL_CONFIG_SET_SIZE;
    case LORA_PACKET_CONFIG_ACK:
        return LORA_PROTOCOL_CONFIG_ACK_SIZE;
    default:
        return 0;
    }
}

static void seal(uint8_t *output, size_t total_size)
{
    put_u16_le(&output[total_size - 2U], crc16_ccitt(output, total_size - 2U));
}

static void encode_uplink_header(uint8_t *output, lora_packet_type_t type,
                                 const lora_uplink_t *uplink)
{
    output[OFF_MAGIC_0] = MAGIC_0;
    output[OFF_MAGIC_1] = MAGIC_1;
    output[OFF_VERSION] = LORA_PROTOCOL_VERSION;
    output[OFF_TYPE] = (uint8_t)type;
    output[OFF_NODE_ID] = uplink->node_id;
    output[OFF_RETRY_COUNT] = uplink->retry_count;
    put_u32_le(&output[OFF_SEQUENCE], uplink->sequence);
    put_u16_le(&output[OFF_CONFIG_REVISION], uplink->config.revision);
    put_u16_le(&output[OFF_SLEEP_S], uplink->config.sleep_s);
    put_u16_le(&output[OFF_WAKE_S], uplink->config.wake_s);
    output[OFF_MEASURE_COUNT] = uplink->config.measure_count;
    put_u32_le(&output[OFF_AWAKE_ELAPSED_MS], uplink->telemetry.awake_elapsed_ms);
    put_u32_le(&output[OFF_LAST_SLEEP_MS], uplink->telemetry.last_sleep_ms);
    put_u64_le(&output[OFF_TOTAL_AWAKE_MS], uplink->telemetry.total_awake_ms);
    put_u64_le(&output[OFF_TOTAL_SLEEP_MS], uplink->telemetry.total_sleep_ms);
    put_u32_le(&output[OFF_WAKE_COUNT], uplink->telemetry.wake_count);
    put_u16_le(&output[OFF_VBAT_MV], uplink->telemetry.vbat_mv);
}

static size_t begin_uplink(uint8_t *output, size_t capacity,
                           const lora_uplink_t *uplink, lora_packet_type_t type)
{
    const size_t size = expected_size(type);
    if (!output || !uplink || capacity < size) {
        return 0;
    }
    memset(output, 0, size);
    encode_uplink_header(output, type, uplink);
    return size;
}

size_t lora_protocol_encode_wake(uint8_t *output, size_t capacity,
                                 const lora_uplink_t *uplink)
{
    const size_t size = begin_uplink(output, capacity, uplink, LORA_PACKET_WAKE);
    if (size == 0) {
        return 0;
    }
    seal(output, size);
    return size;
}

size_t lora_protocol_encode_frame(uint8_t *output, size_t capacity,
                                  const lora_uplink_t *uplink,
                                  uint64_t valid_mask,
                                  const uint16_t *distance_mm)
{
    if (!distance_mm) {
        return 0;
    }
    const size_t size = begin_uplink(output, capacity, uplink, LORA_PACKET_FRAME);
    if (size == 0) {
        return 0;
    }
    output[OFF_ROWS] = 8;
    output[OFF_COLS] = 8;
    put_u64_le(&output[OFF_FRAME_VALID_MASK], valid_mask);
    for (size_t i = 0; i < LORA_PROTOCOL_ZONE_COUNT; ++i) {
        put_u16_le(&output[OFF_FRAME_DISTANCES + (2U * i)], distance_mm[i]);
    }
    seal(output, size);
    return size;
}

size_t lora_protocol_encode_status(uint8_t *output, size_t capacity,
                                   const lora_uplink_t *uplink,
                                   uint8_t status_code)
{
    const size_t size = begin_uplink(output, capacity, uplink, LORA_PACKET_STATUS);
    if (size == 0) {
        return 0;
    }
    output[OFF_STATUS_CODE] = status_code;
    seal(output, size);
    return size;
}

size_t lora_protocol_encode_sleeping(uint8_t *output, size_t capacity,
                                     const lora_uplink_t *uplink,
                                     uint32_t planned_sleep_ms)
{
    const size_t size =
        begin_uplink(output, capacity, uplink, LORA_PACKET_SLEEPING);
    if (size == 0) {
        return 0;
    }
    put_u32_le(&output[OFF_SLEEPING_PLANNED_MS], planned_sleep_ms);
    seal(output, size);
    return size;
}

size_t lora_protocol_encode_config_ack(uint8_t *output, size_t capacity,
                                       const lora_uplink_t *uplink,
                                       uint8_t result)
{
    const size_t size =
        begin_uplink(output, capacity, uplink, LORA_PACKET_CONFIG_ACK);
    if (size == 0) {
        return 0;
    }
    output[OFF_ACK_RESULT] = result;
    seal(output, size);
    return size;
}

size_t lora_protocol_encode_config_set(uint8_t *output, size_t capacity,
                                       uint8_t target_node_id,
                                       const lora_config_t *config)
{
    if (!output || !config || capacity < LORA_PROTOCOL_CONFIG_SET_SIZE ||
        !lora_protocol_config_is_valid(config)) {
        return 0;
    }

    memset(output, 0, LORA_PROTOCOL_CONFIG_SET_SIZE);
    output[OFF_MAGIC_0] = MAGIC_0;
    output[OFF_MAGIC_1] = MAGIC_1;
    output[OFF_VERSION] = LORA_PROTOCOL_VERSION;
    output[OFF_TYPE] = (uint8_t)LORA_PACKET_CONFIG_SET;
    output[OFF_NODE_ID] = target_node_id;
    put_u16_le(&output[OFF_SET_REVISION], config->revision);
    put_u16_le(&output[OFF_SET_SLEEP_S], config->sleep_s);
    put_u16_le(&output[OFF_SET_WAKE_S], config->wake_s);
    output[OFF_SET_MEASURE_COUNT] = config->measure_count;
    seal(output, LORA_PROTOCOL_CONFIG_SET_SIZE);
    return LORA_PROTOCOL_CONFIG_SET_SIZE;
}

int lora_protocol_decode(const uint8_t *data, size_t length,
                         lora_packet_t *packet)
{
    if (!data || !packet || length < LORA_PROTOCOL_CONFIG_SET_SIZE ||
        data[OFF_MAGIC_0] != MAGIC_0 || data[OFF_MAGIC_1] != MAGIC_1 ||
        data[OFF_VERSION] != LORA_PROTOCOL_VERSION) {
        return -1;
    }

    const lora_packet_type_t type = (lora_packet_type_t)data[OFF_TYPE];
    const size_t size = expected_size(type);
    if (size == 0 || length != size ||
        get_u16_le(&data[length - 2U]) != crc16_ccitt(data, length - 2U)) {
        return -1;
    }

    memset(packet, 0, sizeof(*packet));
    packet->type = type;
    packet->node_id = data[OFF_NODE_ID];

    if (type == LORA_PACKET_CONFIG_SET) {
        packet->config.revision = get_u16_le(&data[OFF_SET_REVISION]);
        packet->config.sleep_s = get_u16_le(&data[OFF_SET_SLEEP_S]);
        packet->config.wake_s = get_u16_le(&data[OFF_SET_WAKE_S]);
        packet->config.measure_count = data[OFF_SET_MEASURE_COUNT];
        return lora_protocol_config_is_valid(&packet->config) ? 0 : -1;
    }

    packet->sequence = get_u32_le(&data[OFF_SEQUENCE]);
    packet->retry_count = data[OFF_RETRY_COUNT];
    packet->config.revision = get_u16_le(&data[OFF_CONFIG_REVISION]);
    packet->config.sleep_s = get_u16_le(&data[OFF_SLEEP_S]);
    packet->config.wake_s = get_u16_le(&data[OFF_WAKE_S]);
    packet->config.measure_count = data[OFF_MEASURE_COUNT];
    packet->telemetry.awake_elapsed_ms = get_u32_le(&data[OFF_AWAKE_ELAPSED_MS]);
    packet->telemetry.last_sleep_ms = get_u32_le(&data[OFF_LAST_SLEEP_MS]);
    packet->telemetry.total_awake_ms = get_u64_le(&data[OFF_TOTAL_AWAKE_MS]);
    packet->telemetry.total_sleep_ms = get_u64_le(&data[OFF_TOTAL_SLEEP_MS]);
    packet->telemetry.wake_count = get_u32_le(&data[OFF_WAKE_COUNT]);
    packet->telemetry.vbat_mv = get_u16_le(&data[OFF_VBAT_MV]);

    switch (type) {
    case LORA_PACKET_STATUS:
        packet->status_code = data[OFF_STATUS_CODE];
        break;
    case LORA_PACKET_CONFIG_ACK:
        packet->ack_result = data[OFF_ACK_RESULT];
        break;
    case LORA_PACKET_SLEEPING:
        packet->planned_sleep_ms = get_u32_le(&data[OFF_SLEEPING_PLANNED_MS]);
        break;
    case LORA_PACKET_FRAME:
        if (data[OFF_ROWS] != 8 || data[OFF_COLS] != 8) {
            return -1;
        }
        packet->valid_mask = get_u64_le(&data[OFF_FRAME_VALID_MASK]);
        for (size_t i = 0; i < LORA_PROTOCOL_ZONE_COUNT; ++i) {
            packet->distance_mm[i] =
                get_u16_le(&data[OFF_FRAME_DISTANCES + (2U * i)]);
        }
        break;
    default:
        break;
    }
    return 0;
}

bool lora_protocol_config_is_valid(const lora_config_t *config)
{
    return config && config->sleep_s <= CONFIG_MAX_SLEEP_S &&
           config->wake_s >= 1U && config->wake_s <= CONFIG_MAX_WAKE_S &&
           config->measure_count >= 1U &&
           config->measure_count <= CONFIG_MAX_MEASURE_COUNT;
}

uint32_t lora_protocol_tx_slot_offset_ms(uint8_t node_id)
{
    /* Node 1 transmits immediately, node 2 half a second later, and so on. */
    return node_id > 0U ? (uint32_t)(node_id - 1U) * LORA_PROTOCOL_TX_SLOT_MS
                        : 0U;
}
