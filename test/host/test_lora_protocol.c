#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lora_protocol.h"

static lora_uplink_t make_uplink(uint8_t node_id, uint32_t sequence)
{
    lora_uplink_t uplink;
    memset(&uplink, 0, sizeof(uplink));
    uplink.node_id = node_id;
    uplink.sequence = sequence;
    uplink.retry_count = 2;
    uplink.config.revision = 7;
    uplink.config.sleep_s = 25;
    uplink.config.wake_s = 10;
    uplink.config.measure_count = 1;
    /* Deliberately past the 32-bit range that v1 could not represent. */
    uplink.telemetry.awake_elapsed_ms = 4200;
    uplink.telemetry.last_sleep_ms = 25000;
    uplink.telemetry.total_awake_ms = UINT64_C(0x1234ABCD5678);
    uplink.telemetry.total_sleep_ms = UINT64_C(0xFFFFFFFF0001);
    uplink.telemetry.wake_count = 931;
    uplink.telemetry.vbat_mv = 3920;
    return uplink;
}

static void check_common(const lora_packet_t *decoded, const lora_uplink_t *sent)
{
    assert(decoded->node_id == sent->node_id);
    assert(decoded->sequence == sent->sequence);
    assert(decoded->retry_count == sent->retry_count);
    assert(decoded->config.revision == sent->config.revision);
    assert(decoded->config.sleep_s == sent->config.sleep_s);
    assert(decoded->config.wake_s == sent->config.wake_s);
    assert(decoded->config.measure_count == sent->config.measure_count);
    assert(decoded->telemetry.awake_elapsed_ms == sent->telemetry.awake_elapsed_ms);
    assert(decoded->telemetry.last_sleep_ms == sent->telemetry.last_sleep_ms);
    assert(decoded->telemetry.total_awake_ms == sent->telemetry.total_awake_ms);
    assert(decoded->telemetry.total_sleep_ms == sent->telemetry.total_sleep_ms);
    assert(decoded->telemetry.wake_count == sent->telemetry.wake_count);
    assert(decoded->telemetry.vbat_mv == sent->telemetry.vbat_mv);
}

static void test_frame(void)
{
    uint16_t distances[LORA_PROTOCOL_ZONE_COUNT];
    for (size_t i = 0; i < LORA_PROTOCOL_ZONE_COUNT; ++i) {
        distances[i] = (uint16_t)(200U + i);
    }

    const lora_uplink_t uplink = make_uplink(1, 42);
    /* Extra room so the overlong-length case below stays in bounds. */
    uint8_t encoded[LORA_PROTOCOL_MAX_PACKET_SIZE + 4U];
    const size_t length = lora_protocol_encode_frame(
        encoded, sizeof(encoded), &uplink, UINT64_C(0xA5A5), distances);
    assert(length == LORA_PROTOCOL_FRAME_SIZE);
    assert(length <= 255U); /* SX1262 single-packet limit */

    lora_packet_t decoded;
    assert(lora_protocol_decode(encoded, length, &decoded) == 0);
    assert(decoded.type == LORA_PACKET_FRAME);
    check_common(&decoded, &uplink);
    assert(decoded.valid_mask == UINT64_C(0xA5A5));
    assert(memcmp(decoded.distance_mm, distances, sizeof(distances)) == 0);

    /* A single flipped bit anywhere must fail the CRC. */
    for (size_t i = 0; i < length; ++i) {
        uint8_t corrupted[LORA_PROTOCOL_MAX_PACKET_SIZE];
        memcpy(corrupted, encoded, length);
        corrupted[i] ^= 1U;
        assert(lora_protocol_decode(corrupted, length, &decoded) != 0);
    }

    /* Truncated and overlong buffers must be rejected, not read past. */
    assert(lora_protocol_decode(encoded, length - 1U, &decoded) != 0);
    assert(lora_protocol_decode(encoded, length + 1U, &decoded) != 0);
}

static void test_status_wake_sleeping_ack(void)
{
    const lora_uplink_t uplink = make_uplink(2, 7);
    uint8_t encoded[LORA_PROTOCOL_MAX_PACKET_SIZE];
    lora_packet_t decoded;

    size_t length =
        lora_protocol_encode_status(encoded, sizeof(encoded), &uplink,
                                    LORA_STATUS_TOF_NOT_FOUND);
    assert(length == LORA_PROTOCOL_STATUS_SIZE);
    assert(lora_protocol_decode(encoded, length, &decoded) == 0);
    assert(decoded.type == LORA_PACKET_STATUS);
    assert(decoded.status_code == LORA_STATUS_TOF_NOT_FOUND);
    check_common(&decoded, &uplink);

    length = lora_protocol_encode_wake(encoded, sizeof(encoded), &uplink);
    assert(length == LORA_PROTOCOL_WAKE_SIZE);
    assert(lora_protocol_decode(encoded, length, &decoded) == 0);
    assert(decoded.type == LORA_PACKET_WAKE);
    check_common(&decoded, &uplink);

    length = lora_protocol_encode_sleeping(encoded, sizeof(encoded), &uplink,
                                           25000U);
    assert(length == LORA_PROTOCOL_SLEEPING_SIZE);
    assert(lora_protocol_decode(encoded, length, &decoded) == 0);
    assert(decoded.type == LORA_PACKET_SLEEPING);
    assert(decoded.planned_sleep_ms == 25000U);
    check_common(&decoded, &uplink);

    length = lora_protocol_encode_config_ack(encoded, sizeof(encoded), &uplink,
                                             LORA_ACK_APPLIED_NEXT_CYCLE);
    assert(length == LORA_PROTOCOL_CONFIG_ACK_SIZE);
    assert(lora_protocol_decode(encoded, length, &decoded) == 0);
    assert(decoded.type == LORA_PACKET_CONFIG_ACK);
    assert(decoded.ack_result == LORA_ACK_APPLIED_NEXT_CYCLE);
    check_common(&decoded, &uplink);
}

static void test_config_set(void)
{
    lora_config_t config = {.revision = 9,
                            .sleep_s = 10,
                            .wake_s = 10,
                            .measure_count = 1};
    uint8_t encoded[LORA_PROTOCOL_MAX_PACKET_SIZE];
    lora_packet_t decoded;

    const size_t length =
        lora_protocol_encode_config_set(encoded, sizeof(encoded), 2, &config);
    assert(length == LORA_PROTOCOL_CONFIG_SET_SIZE);
    assert(lora_protocol_decode(encoded, length, &decoded) == 0);
    assert(decoded.type == LORA_PACKET_CONFIG_SET);
    assert(decoded.node_id == 2);
    assert(decoded.config.revision == 9);
    assert(decoded.config.sleep_s == 10);
    assert(decoded.config.wake_s == 10);
    assert(decoded.config.measure_count == 1);

    /* Both demo profiles must survive a round trip. */
    config.sleep_s = 25;
    assert(lora_protocol_encode_config_set(encoded, sizeof(encoded), 1,
                                           &config) ==
           LORA_PROTOCOL_CONFIG_SET_SIZE);
}

static void test_rejects_bad_input(void)
{
    uint8_t encoded[LORA_PROTOCOL_MAX_PACKET_SIZE];
    lora_config_t config = {.revision = 1,
                            .sleep_s = 10,
                            .wake_s = 10,
                            .measure_count = 1};

    /* Out-of-range settings never reach the air. */
    config.wake_s = 0;
    assert(!lora_protocol_config_is_valid(&config));
    assert(lora_protocol_encode_config_set(encoded, sizeof(encoded), 1, &config) == 0);
    config.wake_s = 10;

    config.measure_count = 0;
    assert(!lora_protocol_config_is_valid(&config));
    config.measure_count = 11;
    assert(!lora_protocol_config_is_valid(&config));
    config.measure_count = 1;

    config.sleep_s = 3601;
    assert(!lora_protocol_config_is_valid(&config));
    config.sleep_s = 0; /* continuous operation is allowed */
    assert(lora_protocol_config_is_valid(&config));

    /* Too small a buffer must fail instead of overflowing. */
    const lora_uplink_t uplink = make_uplink(1, 1);
    uint16_t distances[LORA_PROTOCOL_ZONE_COUNT] = {0};
    assert(lora_protocol_encode_frame(encoded, LORA_PROTOCOL_FRAME_SIZE - 1U,
                                      &uplink, 0, distances) == 0);
    assert(lora_protocol_encode_wake(encoded, LORA_PROTOCOL_WAKE_SIZE - 1U,
                                     &uplink) == 0);

    /* Unknown type and wrong version are rejected. */
    size_t length = lora_protocol_encode_wake(encoded, sizeof(encoded), &uplink);
    lora_packet_t decoded;
    uint8_t corrupted[LORA_PROTOCOL_MAX_PACKET_SIZE];

    memcpy(corrupted, encoded, length);
    corrupted[3] = 99;
    assert(lora_protocol_decode(corrupted, length, &decoded) != 0);

    memcpy(corrupted, encoded, length);
    corrupted[2] = 1; /* the retired v1 */
    assert(lora_protocol_decode(corrupted, length, &decoded) != 0);

    memcpy(corrupted, encoded, length);
    corrupted[0] = 'X';
    assert(lora_protocol_decode(corrupted, length, &decoded) != 0);
}

static void test_tx_slots(void)
{
    /* B-1 and B-2 must never land on the same offset. */
    assert(lora_protocol_tx_slot_offset_ms(1) !=
           lora_protocol_tx_slot_offset_ms(2));
    assert(lora_protocol_tx_slot_offset_ms(1) == 0U);
    assert(lora_protocol_tx_slot_offset_ms(2) == LORA_PROTOCOL_TX_SLOT_MS);
}

int main(void)
{
    test_frame();
    test_status_wake_sleeping_ack();
    test_config_set();
    test_rejects_bad_input();
    test_tx_slots();
    puts("lora protocol v2 tests passed");
    return 0;
}
