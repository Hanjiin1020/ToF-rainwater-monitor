#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "bridge.h"
#include "lora_protocol.h"
#include "lora_radio.h"

static const char *TAG = "bridge";

/* Node IDs are 1-based and fixed by SPEC: B-1 = 1, B-2 = 2. */
#define BRIDGE_FIRST_NODE_ID 1U
#define BRIDGE_NODE_COUNT 2U

#define COMMAND_LINE_MAX 96U
#define CONFIG_SET_ATTEMPTS 4U

/* Events the Mac UI consumes. The version prefix lets the UI reject a firmware
 * it does not understand instead of silently misreading fields. */
#define EVENT_SCHEMA_VERSION 2

typedef enum {
    CFG_IDLE = 0,   /* nothing to push */
    CFG_PENDING,    /* waiting for the node to wake, or for its ACK */
    CFG_ACKED,      /* node confirmed; it applies from its next cycle */
    CFG_FAILED,     /* gave up after CONFIG_SET_ATTEMPTS wakes */
} config_state_t;

typedef struct {
    lora_config_t desired;
    config_state_t state;
    uint8_t attempts;
    bool ever_seen;
    uint32_t last_wake_count;
    uint32_t ack_wake_count;
    bool have_wake_count;
    /* Highest revision the node has reported running. A keeps its own counter
     * in RAM only, so this is the one place a bridge restart can recover it
     * from. */
    uint16_t last_reported_rev;
} node_state_t;

static node_state_t g_nodes[BRIDGE_NODE_COUNT];

/* A frame carries 64 numbers, so printing the readable log and the JSON at the
 * same time buries the prompt. Humans get the readable form by default and the
 * UI switches to JSON on connect. */
static bool g_emit_human = true;
static bool g_emit_json;

static node_state_t *node_slot(uint8_t node_id)
{
    if (node_id < BRIDGE_FIRST_NODE_ID ||
        node_id >= BRIDGE_FIRST_NODE_ID + BRIDGE_NODE_COUNT) {
        return NULL;
    }
    return &g_nodes[node_id - BRIDGE_FIRST_NODE_ID];
}

static const char *status_name(uint8_t status)
{
    switch (status) {
    case LORA_STATUS_BOOT:
        return "BOOT";
    case LORA_STATUS_TOF_NOT_FOUND:
        return "TOF_NOT_FOUND";
    case LORA_STATUS_TOF_INIT_FAILED:
        return "TOF_INIT_FAILED";
    case LORA_STATUS_TOF_READ_FAILED:
        return "TOF_READ_FAILED";
    case LORA_STATUS_TOF_SLEEP_FAILED:
        return "TOF_SLEEP_FAILED";
    case LORA_STATUS_RADIO_SLEEP_FAILED:
        return "RADIO_SLEEP_FAILED";
    default:
        return "UNKNOWN";
    }
}

static const char *type_name(lora_packet_type_t type)
{
    switch (type) {
    case LORA_PACKET_WAKE:
        return "WAKE";
    case LORA_PACKET_FRAME:
        return "FRAME";
    case LORA_PACKET_STATUS:
        return "STATUS";
    case LORA_PACKET_SLEEPING:
        return "SLEEPING";
    case LORA_PACKET_CONFIG_ACK:
        return "CONFIG_ACK";
    default:
        return "UNKNOWN";
    }
}

static const char *config_state_name(config_state_t state)
{
    switch (state) {
    case CFG_PENDING:
        return "pending";
    case CFG_ACKED:
        return "acked";
    case CFG_FAILED:
        return "failed";
    default:
        return "idle";
    }
}

/* ---------------- output to the Mac ---------------- */

/* One JSON object per line so the UI can read it with readline(). */
static void emit_packet_event(const lora_packet_t *packet,
                              const lora_rx_metadata_t *metadata)
{
    if (!g_emit_json) {
        return;
    }
    printf("{\"v\":%d,\"ev\":\"%s\",\"node\":%u,\"seq\":%" PRIu32
           ",\"rssi\":%d,\"snr\":%d,\"rev\":%u,\"sleep_s\":%u,\"wake_s\":%u"
           ",\"measure\":%u,\"retries\":%u,\"awake_ms\":%" PRIu32
           ",\"last_sleep_ms\":%" PRIu32 ",\"total_awake_ms\":%llu"
           ",\"total_sleep_ms\":%llu,\"wakes\":%" PRIu32 ",\"vbat_mv\":%u",
           EVENT_SCHEMA_VERSION, type_name(packet->type), packet->node_id,
           packet->sequence, metadata->rssi_dbm, metadata->snr_db,
           packet->config.revision, packet->config.sleep_s,
           packet->config.wake_s, packet->config.measure_count,
           packet->retry_count, packet->telemetry.awake_elapsed_ms,
           packet->telemetry.last_sleep_ms,
           (unsigned long long)packet->telemetry.total_awake_ms,
           (unsigned long long)packet->telemetry.total_sleep_ms,
           packet->telemetry.wake_count, packet->telemetry.vbat_mv);

    switch (packet->type) {
    case LORA_PACKET_STATUS:
        printf(",\"status\":%u,\"status_name\":\"%s\"", packet->status_code,
               status_name(packet->status_code));
        break;
    case LORA_PACKET_SLEEPING:
        printf(",\"planned_sleep_ms\":%" PRIu32, packet->planned_sleep_ms);
        break;
    case LORA_PACKET_CONFIG_ACK:
        printf(",\"ack_result\":%u", packet->ack_result);
        break;
    case LORA_PACKET_FRAME:
        printf(",\"valid\":%u,\"valid_mask\":%llu,\"d\":[",
               (unsigned)__builtin_popcountll(packet->valid_mask),
               (unsigned long long)packet->valid_mask);
        for (size_t zone = 0; zone < LORA_PROTOCOL_ZONE_COUNT; ++zone) {
            printf(zone == 0 ? "%u" : ",%u", packet->distance_mm[zone]);
        }
        printf("]");
        break;
    default:
        break;
    }
    printf("}\n");
    fflush(stdout);
}

static void emit_config_event(uint8_t node_id, const node_state_t *node,
                              const char *note)
{
    printf("{\"v\":%d,\"ev\":\"config\",\"node\":%u,\"state\":\"%s\""
           ",\"rev\":%u,\"sleep_s\":%u,\"wake_s\":%u,\"measure\":%u"
           ",\"attempts\":%u,\"note\":\"%s\"}\n",
           EVENT_SCHEMA_VERSION, node_id, config_state_name(node->state),
           node->desired.revision, node->desired.sleep_s, node->desired.wake_s,
           node->desired.measure_count, node->attempts, note);
    fflush(stdout);
}

/* The readable form, for a plain idf.py monitor session. */
static void print_human(const lora_packet_t *packet,
                        const lora_rx_metadata_t *metadata)
{
    if (!g_emit_human) {
        return;
    }
    ESP_LOGI(TAG,
             "RX B-%u %s seq=%" PRIu32 " rev=%u cfg=%us/%us/x%u"
             " RSSI=%d dBm SNR=%d dB",
             packet->node_id, type_name(packet->type), packet->sequence,
             packet->config.revision, packet->config.sleep_s,
             packet->config.wake_s, packet->config.measure_count,
             metadata->rssi_dbm, metadata->snr_db);
    ESP_LOGI(TAG,
             "   awake=%" PRIu32 "ms last_sleep=%" PRIu32
             "ms total_awake=%llums total_sleep=%llums wakes=%" PRIu32
             " retries=%u vbat=%umV",
             packet->telemetry.awake_elapsed_ms, packet->telemetry.last_sleep_ms,
             (unsigned long long)packet->telemetry.total_awake_ms,
             (unsigned long long)packet->telemetry.total_sleep_ms,
             packet->telemetry.wake_count, packet->retry_count,
             packet->telemetry.vbat_mv);

    if (packet->type == LORA_PACKET_STATUS) {
        ESP_LOGI(TAG, "   status=%s(%u)", status_name(packet->status_code),
                 packet->status_code);
        return;
    }
    if (packet->type == LORA_PACKET_SLEEPING) {
        ESP_LOGI(TAG, "   planned_sleep=%" PRIu32 "ms",
                 packet->planned_sleep_ms);
        return;
    }
    if (packet->type != LORA_PACKET_FRAME) {
        return;
    }
    for (size_t row = 0; row < 8; ++row) {
        printf("B-%u R%u |", packet->node_id, (unsigned)row);
        for (size_t column = 0; column < 8; ++column) {
            const size_t zone = row * 8U + column;
            if ((packet->valid_mask & (UINT64_C(1) << zone)) != 0) {
                printf(" %4u", packet->distance_mm[zone]);
            } else {
                printf(" ----");
            }
        }
        putchar('\n');
    }
    fflush(stdout);
}

/* ---------------- config downlink ---------------- */

static void send_config_set(uint8_t node_id, node_state_t *node)
{
    uint8_t packet[LORA_PROTOCOL_CONFIG_SET_SIZE];
    const size_t length = lora_protocol_encode_config_set(
        packet, sizeof(packet), node_id, &node->desired);
    if (length == 0) {
        ESP_LOGE(TAG, "CONFIG_SET encode failed for B-%u", node_id);
        node->state = CFG_FAILED;
        emit_config_event(node_id, node, "encode_failed");
        return;
    }

    node->attempts++;
    const esp_err_t err = lora_radio_send(packet, length);
    ESP_LOGI(TAG, "-> B-%u CONFIG_SET rev=%u sleep=%us wake=%us x%u (try %u/%u) %s",
             node_id, node->desired.revision, node->desired.sleep_s,
             node->desired.wake_s, node->desired.measure_count, node->attempts,
             CONFIG_SET_ATTEMPTS, esp_err_to_name(err));

    if (node->attempts >= CONFIG_SET_ATTEMPTS) {
        /* The node keeps waking without acknowledging, so stop shouting at it
         * and let the UI decide what to do. */
        node->state = CFG_FAILED;
        emit_config_event(node_id, node, "no_ack");
    }
}

/* Settings live only in the node's RAM, so a restart silently drops them back
 * to the boot defaults. Detect that from wake_count going backwards rather
 * than from the revision: a node that has just acknowledged still reports the
 * old revision until its next cycle, and treating that as a restart re-sends
 * the config for nothing. */
static void reconcile_revision(uint8_t node_id, node_state_t *node,
                               const lora_packet_t *packet)
{
    const uint32_t wakes = packet->telemetry.wake_count;
    const bool restarted = node->have_wake_count && wakes < node->last_wake_count;

    node->last_wake_count = wakes;
    node->have_wake_count = true;

    /* Uplinks other than CONFIG_ACK always carry the config actually in force,
     * so this is trustworthy. The ACK's header is not: older senders put the
     * applied revision there rather than the acknowledged one, which is why
     * handle_packet() returns before reaching here for those. */
    if (packet->config.revision > node->last_reported_rev) {
        node->last_reported_rev = packet->config.revision;
    }

    if (node->desired.revision == 0) {
        return; /* nothing has been pushed to this node yet */
    }

    if (restarted) {
        ESP_LOGW(TAG, "B-%u restarted (wakes reset), re-queuing rev=%u", node_id,
                 node->desired.revision);
        node->state = CFG_PENDING;
        node->attempts = 0;
        emit_config_event(node_id, node, "node_restarted");
        return;
    }

    /* Acknowledged but still running the old settings two wakes later means the
     * node never actually adopted them. */
    if (node->state == CFG_ACKED &&
        packet->config.revision < node->desired.revision &&
        wakes >= node->ack_wake_count + 2U) {
        ESP_LOGW(TAG, "B-%u acked rev=%u but still reports rev=%u, re-queuing",
                 node_id, node->desired.revision, packet->config.revision);
        node->state = CFG_PENDING;
        node->attempts = 0;
        emit_config_event(node_id, node, "not_applied");
    }
}

static void handle_packet(const lora_packet_t *packet,
                          const lora_rx_metadata_t *metadata)
{
    print_human(packet, metadata);
    emit_packet_event(packet, metadata);

    node_state_t *node = node_slot(packet->node_id);
    if (!node) {
        ESP_LOGW(TAG, "packet from unknown node %u", packet->node_id);
        return;
    }
    node->ever_seen = true;

    if (packet->type == LORA_PACKET_CONFIG_ACK) {
        /* Older sender builds put the revision still in force in the header
         * rather than the one being acknowledged, so a positive result while a
         * push is outstanding is what counts. reconcile_revision() catches the
         * case where the node never actually adopts it. */
        if (node->state == CFG_PENDING &&
            packet->ack_result == LORA_ACK_APPLIED_NEXT_CYCLE) {
            node->state = CFG_ACKED;
            node->ack_wake_count = packet->telemetry.wake_count;
            emit_config_event(packet->node_id, node, "acked");
            ESP_LOGI(TAG, "B-%u acked rev=%u", packet->node_id,
                     node->desired.revision);
        } else {
            ESP_LOGW(TAG, "B-%u rejected rev=%u (result=%u)", packet->node_id,
                     packet->config.revision, packet->ack_result);
        }
        return;
    }

    reconcile_revision(packet->node_id, node, packet);

    /* WAKE is the one moment the node is guaranteed to be listening. */
    if (packet->type == LORA_PACKET_WAKE && node->state == CFG_PENDING) {
        send_config_set(packet->node_id, node);
    }
}

/* ---------------- commands from the Mac ---------------- */

static void queue_config(uint8_t node_id, uint16_t sleep_s, uint16_t wake_s,
                         uint8_t measure_count)
{
    node_state_t *node = node_slot(node_id);
    if (!node) {
        ESP_LOGE(TAG, "CFG: node %u out of range", node_id);
        return;
    }

    /* The node rejects anything below the revision it is already running, and
     * A's counter lives in RAM: reflash or unplug the bridge and it restarts at
     * zero while the node still holds the number from the previous session.
     * Every push would then come back REJECTED_STALE, one wake cycle at a time
     * - punishing at a 25 s cycle, unusable at an hour. Start from whichever of
     * the two is ahead. */
    uint16_t base = node->desired.revision;
    if (node->last_reported_rev > base) {
        ESP_LOGI(TAG, "B-%u is running rev=%u, ahead of ours (%u); skipping past it",
                 node_id, node->last_reported_rev, base);
        base = node->last_reported_rev;
    }

    lora_config_t wanted = {
        .revision = (uint16_t)(base + 1U),
        .sleep_s = sleep_s,
        .wake_s = wake_s,
        .measure_count = measure_count,
    };
    if (!lora_protocol_config_is_valid(&wanted)) {
        ESP_LOGE(TAG, "CFG: values out of range for B-%u", node_id);
        return;
    }

    node->desired = wanted;
    node->state = CFG_PENDING;
    node->attempts = 0;
    ESP_LOGI(TAG, "queued for B-%u: rev=%u sleep=%us wake=%us x%u", node_id,
             wanted.revision, sleep_s, wake_s, measure_count);
    emit_config_event(node_id, node, "queued");
}

static void report_status(void)
{
    for (unsigned i = 0; i < BRIDGE_NODE_COUNT; ++i) {
        const uint8_t node_id = (uint8_t)(BRIDGE_FIRST_NODE_ID + i);
        emit_config_event(node_id, &g_nodes[i],
                          g_nodes[i].ever_seen ? "seen" : "never_seen");
    }
}

static void handle_command(char *line)
{
    unsigned node_id = 0;
    unsigned sleep_s = 0;
    unsigned wake_s = 0;
    unsigned measure_count = 0;

    if (sscanf(line, "CFG %u %u %u %u", &node_id, &sleep_s, &wake_s,
               &measure_count) == 4) {
        queue_config((uint8_t)node_id, (uint16_t)sleep_s, (uint16_t)wake_s,
                     (uint8_t)measure_count);
        return;
    }
    if (strncmp(line, "STATUS", 6) == 0) {
        report_status();
        return;
    }
    if (strncmp(line, "MODE ", 5) == 0) {
        const char *mode = line + 5;
        if (strncmp(mode, "HUMAN", 5) == 0) {
            g_emit_human = true;
            g_emit_json = false;
        } else if (strncmp(mode, "JSON", 4) == 0) {
            g_emit_human = false;
            g_emit_json = true;
        } else if (strncmp(mode, "BOTH", 4) == 0) {
            g_emit_human = true;
            g_emit_json = true;
        } else {
            ESP_LOGW(TAG, "MODE takes HUMAN, JSON or BOTH");
            return;
        }
        ESP_LOGI(TAG, "output mode: human=%d json=%d", g_emit_human,
                 g_emit_json);
        return;
    }
    ESP_LOGW(TAG, "unknown command: '%s'. Use CFG <node> <sleep_s> <wake_s>"
                  " <measure> | STATUS | MODE HUMAN|JSON|BOTH", line);
}

/* Reads without blocking so the radio keeps being polled between characters. */
static void poll_commands(void)
{
    static char line[COMMAND_LINE_MAX];
    static size_t length;

    uint8_t ch;
    while (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &ch, 1, 0) == 1) {
        if (ch == '\r' || ch == '\n') {
            if (length > 0) {
                line[length] = '\0';
                handle_command(line);
                length = 0;
            }
            continue;
        }
        if (length + 1 < sizeof(line)) {
            line[length++] = (char)ch;
        } else {
            ESP_LOGW(TAG, "command too long, dropped");
            length = 0;
        }
    }
}

void run_bridge(void)
{
    ESP_LOGI(TAG, "role=A bridge, frequency=%lu Hz", LORA_RADIO_FREQUENCY_HZ);

    /* stdin is not readable until the UART driver owns the console port. */
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 512, 0, 0,
                                        NULL, 0));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    ESP_ERROR_CHECK(lora_radio_init());
    ESP_LOGI(TAG, "ready. commands: CFG <node> <sleep_s> <wake_s> <measure>"
                  " | STATUS | MODE HUMAN|JSON|BOTH");
    report_status();

    while (true) {
        poll_commands();

        uint8_t encoded[LORA_PROTOCOL_MAX_PACKET_SIZE];
        size_t length = 0;
        lora_rx_metadata_t metadata = {0};
        const esp_err_t err =
            lora_radio_receive(encoded, sizeof(encoded), &length, &metadata);
        if (err != ESP_OK) {
            if (err != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "radio receive error: %s", esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(LORA_RECEIVE_POLL_MS));
            continue;
        }

        lora_packet_t packet;
        if (lora_protocol_decode(encoded, length, &packet) != 0) {
            ESP_LOGW(TAG, "discarded invalid packet (%u bytes)",
                     (unsigned)length);
            continue;
        }
        handle_packet(&packet, &metadata);
    }
}
