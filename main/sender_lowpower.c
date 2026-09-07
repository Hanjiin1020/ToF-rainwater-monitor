#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_uart.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "lora_protocol.h"
#include "lora_radio.h"
#include "sender_lowpower.h"
#include "tof_driver.h"

static const char *TAG = "lp_sender";

/* Short listen right after WAKE so a config queued by A is applied before the
 * measurement rather than a cycle later. */
#define EARLY_CONFIG_WINDOW_MS 400U
#define RX_POLL_MS 10U
#define SENSOR_SETTLE_MS 10U
#define BOOT_SETTLE_MS 500U

/* Heltec V3 battery sense: pulling GPIO37 low connects VBAT to GPIO1 through a
 * 390k/100k divider, so the cell voltage is the reading times 4.9. The control
 * pin is released again afterwards so the divider does not sit there draining
 * the battery between measurements. */
#define VBAT_ADC_CTRL_GPIO GPIO_NUM_37
#define VBAT_ADC_CHANNEL ADC_CHANNEL_0 /* GPIO1 on ESP32-S3 */
#define VBAT_DIVIDER_NUMERATOR 490
#define VBAT_DIVIDER_DENOMINATOR 100
#define VBAT_SETTLE_MS 10U
#define VBAT_SAMPLES 8
/* A 1S Li-Po outside this range means the reading is not the battery. */
#define VBAT_PLAUSIBLE_MIN_MV 2500
#define VBAT_PLAUSIBLE_MAX_MV 4600
/* Long enough for the carrier rail to fully discharge and the sensor to boot. */
#define SENSOR_POWER_OFF_MS 300U
#define SENSOR_POWER_ON_MS 100U

/* Descending order. 1 MHz is left out: it failed on this harness while the
 * diagnostic succeeded at 100 kHz, and 400 kHz already uploads the ~84 KB
 * sensor firmware fast enough for a 10 s wake window. 50 kHz is the last
 * resort: it takes several seconds to upload but tolerates the worst wiring. */
static const uint32_t k_i2c_speeds_hz[] = {400000U, 100000U, 50000U};

typedef struct {
    uint8_t node_id;
    uint32_t sequence;

    lora_config_t applied;
    lora_config_t pending;
    bool has_pending;

    lora_telemetry_t telemetry;
    int64_t wake_start_us;
    uint8_t attempts_used; /* measurement retries burned in this wake window */

    tof_sensor_t sensor;
    uint32_t i2c_speed_hz; /* 0 until one is known to work */
    bool sensor_ready; /* initialised at least once; may be asleep */
    bool ranging;
} lp_state_t;

static adc_oneshot_unit_handle_t g_adc;
static adc_cali_handle_t g_adc_cali;

static void vbat_init(void)
{
    const gpio_config_t control = {
        .pin_bit_mask = UINT64_C(1) << VBAT_ADC_CTRL_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    const esp_err_t gpio_err = gpio_config(&control);
    if (gpio_err != ESP_OK) {
        ESP_LOGE(TAG, "vbat: GPIO%d config failed: %s", VBAT_ADC_CTRL_GPIO,
                 esp_err_to_name(gpio_err));
        return;
    }
    gpio_set_level(VBAT_ADC_CTRL_GPIO, 1); /* divider off until measuring */

    const adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    esp_err_t err = adc_oneshot_new_unit(&unit, &g_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "vbat: adc_oneshot_new_unit failed: %s",
                 esp_err_to_name(err));
        g_adc = NULL;
        return;
    }
    const adc_oneshot_chan_cfg_t channel = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(g_adc, VBAT_ADC_CHANNEL, &channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "vbat: adc channel config failed: %s",
                 esp_err_to_name(err));
        adc_oneshot_del_unit(g_adc);
        g_adc = NULL;
        return;
    }

    /* Without calibration the raw counts are only roughly proportional to
     * volts; with it the reading is good enough to trust a low-battery call. */
    const adc_cali_curve_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .chan = VBAT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali, &g_adc_cali);
    if (err != ESP_OK) {
        g_adc_cali = NULL;
        ESP_LOGW(TAG, "vbat: ADC calibration unavailable (%s); raw counts are "
                      "not millivolts, so readings will be rejected",
                 esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "vbat: ready (ctrl=GPIO%d, ADC1 ch%d, cali=%s)",
             VBAT_ADC_CTRL_GPIO, (int)VBAT_ADC_CHANNEL,
             g_adc_cali ? "yes" : "no");
}

/* Returns 0 when the battery cannot be read, which the UI shows as unknown
 * rather than as a wrong number. */
static uint16_t vbat_read_mv(void)
{
    if (!g_adc) {
        return 0;
    }
    static bool explained;
    gpio_set_level(VBAT_ADC_CTRL_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(VBAT_SETTLE_MS));

    int total = 0;
    int taken = 0;
    for (int i = 0; i < VBAT_SAMPLES; ++i) {
        int raw = 0;
        if (adc_oneshot_read(g_adc, VBAT_ADC_CHANNEL, &raw) != ESP_OK) {
            continue;
        }
        int mv = raw;
        if (g_adc_cali &&
            adc_cali_raw_to_voltage(g_adc_cali, raw, &mv) != ESP_OK) {
            continue;
        }
        total += mv;
        ++taken;
    }
    gpio_set_level(VBAT_ADC_CTRL_GPIO, 1);

    if (taken == 0) {
        ESP_LOGW(TAG, "vbat: every ADC read failed");
        return 0;
    }
    const int at_pin_mv = total / taken;
    const int battery_mv =
        at_pin_mv * VBAT_DIVIDER_NUMERATOR / VBAT_DIVIDER_DENOMINATOR;

    /* Reported once so a wrong pin or a broken divider is visible on the
     * console instead of silently becoming "unknown". */
    if (!explained) {
        explained = true;
        ESP_LOGI(TAG, "vbat: pin=%d mV, battery=%d mV (%d samples)", at_pin_mv,
                 battery_mv, taken);
    }

    if (battery_mv < VBAT_PLAUSIBLE_MIN_MV ||
        battery_mv > VBAT_PLAUSIBLE_MAX_MV) {
        ESP_LOGW(TAG, "vbat: %d mV outside %d..%d, reporting unknown",
                 battery_mv, VBAT_PLAUSIBLE_MIN_MV, VBAT_PLAUSIBLE_MAX_MV);
        return 0;
    }
    return (uint16_t)battery_mv;
}

static int64_t now_us(void)
{
    return esp_timer_get_time();
}

static uint32_t awake_elapsed_ms(const lp_state_t *state)
{
    return (uint32_t)((now_us() - state->wake_start_us) / 1000);
}

static uint32_t window_remaining_ms(const lp_state_t *state)
{
    const uint32_t window_ms = (uint32_t)state->applied.wake_s * 1000U;
    const uint32_t elapsed = awake_elapsed_ms(state);
    return elapsed >= window_ms ? 0U : window_ms - elapsed;
}

static lora_uplink_t uplink_of(lp_state_t *state)
{
    lora_telemetry_t telemetry = state->telemetry;
    telemetry.awake_elapsed_ms = awake_elapsed_ms(state);
    return (lora_uplink_t){
        .node_id = state->node_id,
        .sequence = state->sequence++,
        .retry_count = state->attempts_used,
        .config = state->applied,
        .telemetry = telemetry,
    };
}

static void send_encoded(const uint8_t *packet, size_t length, const char *what)
{
    if (length == 0) {
        ESP_LOGE(TAG, "%s encode failed", what);
        return;
    }
    const esp_err_t err = lora_radio_send(packet, length);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s tx failed: %s", what, esp_err_to_name(err));
    }
}

static void send_status(lp_state_t *state, uint8_t status_code)
{
    uint8_t packet[LORA_PROTOCOL_STATUS_SIZE];
    const lora_uplink_t uplink = uplink_of(state);
    send_encoded(packet,
                 lora_protocol_encode_status(packet, sizeof(packet), &uplink,
                                             status_code),
                 "status");
}

static void send_wake(lp_state_t *state)
{
    uint8_t packet[LORA_PROTOCOL_WAKE_SIZE];
    const lora_uplink_t uplink = uplink_of(state);
    send_encoded(packet,
                 lora_protocol_encode_wake(packet, sizeof(packet), &uplink),
                 "wake");
}

static void send_sleeping(lp_state_t *state, uint32_t planned_sleep_ms)
{
    uint8_t packet[LORA_PROTOCOL_SLEEPING_SIZE];
    const lora_uplink_t uplink = uplink_of(state);
    send_encoded(packet,
                 lora_protocol_encode_sleeping(packet, sizeof(packet), &uplink,
                                               planned_sleep_ms),
                 "sleeping");
}

/* The common header would otherwise carry the revision still in force, which
 * tells A nothing about which CONFIG_SET this answers. Name the acknowledged
 * revision instead. */
static void send_config_ack(lp_state_t *state, uint8_t result,
                            uint16_t acked_revision)
{
    uint8_t packet[LORA_PROTOCOL_CONFIG_ACK_SIZE];
    lora_uplink_t uplink = uplink_of(state);
    uplink.config.revision = acked_revision;
    send_encoded(packet,
                 lora_protocol_encode_config_ack(packet, sizeof(packet),
                                                 &uplink, result),
                 "config_ack");
}

/* A retries CONFIG_SET when an ACK is lost, so re-sending a revision that is
 * already applied or already queued must ACK positively again rather than look
 * like a stale command. Only strictly older revisions are rejected. */
static void handle_config_set(lp_state_t *state, const lora_packet_t *packet)
{
    const uint16_t revision = packet->config.revision;

    if (!lora_protocol_config_is_valid(&packet->config)) {
        ESP_LOGW(TAG, "config revision %u out of range", revision);
        send_config_ack(state, LORA_ACK_REJECTED_RANGE, revision);
        return;
    }

    if (revision == state->applied.revision ||
        (state->has_pending && revision == state->pending.revision)) {
        ESP_LOGI(TAG, "config revision %u already known, re-acking", revision);
        send_config_ack(state, LORA_ACK_APPLIED_NEXT_CYCLE, revision);
        return;
    }

    if (revision < state->applied.revision) {
        ESP_LOGW(TAG, "config revision %u older than applied %u", revision,
                 state->applied.revision);
        send_config_ack(state, LORA_ACK_REJECTED_STALE, revision);
        return;
    }

    state->pending = packet->config;
    state->has_pending = true;
    ESP_LOGI(TAG, "queued revision %u: sleep=%us wake=%us measure=%u", revision,
             packet->config.sleep_s, packet->config.wake_s,
             packet->config.measure_count);
    send_config_ack(state, LORA_ACK_APPLIED_NEXT_CYCLE, revision);
}

static void service_config(lp_state_t *state, uint32_t budget_ms)
{
    const int64_t deadline = now_us() + (int64_t)budget_ms * 1000;
    while (now_us() < deadline) {
        uint8_t buffer[LORA_PROTOCOL_MAX_PACKET_SIZE];
        size_t length = 0;
        if (lora_radio_receive(buffer, sizeof(buffer), &length, NULL) !=
            ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(RX_POLL_MS));
            continue;
        }
        lora_packet_t packet;
        if (lora_protocol_decode(buffer, length, &packet) != 0) {
            continue;
        }
        /* Ignore the other node's traffic and our own echoes. */
        if (packet.type != LORA_PACKET_CONFIG_SET ||
            packet.node_id != state->node_id) {
            continue;
        }
        handle_config_set(state, &packet);
    }
}

/* tof_deinit() only tears down the I2C side. Once the sensor browns out - the
 * VCSEL draws ~150 mA the instant ranging starts - it stops answering even
 * is_alive, and no amount of re-init recovers it. Cutting VIN is the only way
 * back. This works because the carrier is fed from Vext; on a node wired to the
 * fixed 3V3 rail it is a harmless no-op. */
static void power_cycle_sensor(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = UINT64_C(1) << BOARD_VEXT_CONTROL_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&config) != ESP_OK) {
        return;
    }
    ESP_LOGW(TAG, "power-cycling the sensor through Vext");
    gpio_set_level(BOARD_VEXT_CONTROL_GPIO, !BOARD_VEXT_ENABLED_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_POWER_OFF_MS));
    gpio_set_level(BOARD_VEXT_CONTROL_GPIO, BOARD_VEXT_ENABLED_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_POWER_ON_MS));
}

static size_t speed_count(void)
{
    return sizeof(k_i2c_speeds_hz) / sizeof(k_i2c_speeds_hz[0]);
}

/* Try whatever worked last time first, then the rest in order. */
static uint32_t speed_for_attempt(const lp_state_t *state, size_t index)
{
    if (state->i2c_speed_hz == 0) {
        return k_i2c_speeds_hz[index];
    }
    if (index == 0) {
        return state->i2c_speed_hz;
    }
    const uint32_t candidate = k_i2c_speeds_hz[index - 1];
    return candidate == state->i2c_speed_hz ? k_i2c_speeds_hz[index]
                                            : candidate;
}

/* The harness does not tolerate the same clock on every build: 1 MHz was
 * silently failing where 100 kHz worked. These nodes sit inside a sealed
 * cartridge, so probe downwards and remember what answered instead of baking
 * one speed in. */
static bool try_init_speeds(lp_state_t *state, const tof_config_t *base)
{
    for (size_t i = 0; !state->sensor_ready && i < speed_count(); ++i) {
        const uint32_t speed_hz = speed_for_attempt(state, i);
        tof_config_t attempt_config = *base;
        attempt_config.i2c_speed_hz = speed_hz;

        const int64_t started_us = now_us();
        const esp_err_t err = tof_init(&state->sensor, &attempt_config);
        if (err == ESP_OK) {
            state->sensor_ready = true;
            state->i2c_speed_hz = speed_hz;
            ESP_LOGI(TAG, "ToF init ok at %u kHz in %" PRIu32 " ms",
                     (unsigned)(speed_hz / 1000U),
                     (uint32_t)((now_us() - started_us) / 1000));
            break;
        }
        state->attempts_used++;
        ESP_LOGW(TAG, "tof_init at %u kHz failed: %s",
                 (unsigned)(speed_hz / 1000U), esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(TOF_INIT_RETRY_DELAY_MS));
    }

    return state->sensor_ready;
}

/* Brings the sensor from SLEEP (or from nothing at all) back to ranging. */
static bool prepare_sensor(lp_state_t *state)
{
    const tof_config_t config = app_default_tof_config();

    if (state->sensor_ready) {
        const esp_err_t err = tof_wakeup(&state->sensor);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "tof_wakeup failed (%s), reinitialising",
                     esp_err_to_name(err));
            tof_deinit(&state->sensor);
            state->sensor_ready = false;
        } else {
            vTaskDelay(pdMS_TO_TICKS(SENSOR_SETTLE_MS));
        }
    }

    if (!try_init_speeds(state, &config)) {
        /* Nothing answered on any clock, so the sensor is wedged rather than
         * just slow. Cut its power and give it one clean start. */
        power_cycle_sensor();
        try_init_speeds(state, &config);
    }

    if (!state->sensor_ready) {
        send_status(state, LORA_STATUS_TOF_NOT_FOUND);
        return false;
    }

    if (tof_start_ranging(&state->sensor) != ESP_OK) {
        /* Starting ranging is the moment the VCSEL lights up, so this is where
         * a weak supply collapses and leaves the sensor unresponsive. Cut power
         * now rather than spending the next cycles on init attempts that can
         * only return ESP_ERR_NOT_FOUND. */
        ESP_LOGE(TAG, "tof_start_ranging failed");
        send_status(state, LORA_STATUS_TOF_INIT_FAILED);
        tof_deinit(&state->sensor);
        state->sensor_ready = false;
        power_cycle_sensor();
        return false;
    }
    state->ranging = true;

    /* Ranging restarts every wake, and the first frame it produces has never
     * settled: every observed cycle reported zero valid zones on it and real
     * data on the next one. Drop it here so it does not eat a measurement
     * retry. */
    tof_frame_t settling_frame;
    (void)tof_get_frame(&state->sensor, &settling_frame,
                        pdMS_TO_TICKS(TOF_FRAME_TIMEOUT_MS));
    return true;
}

static uint64_t frame_valid_mask(const tof_frame_t *frame)
{
    uint64_t valid_mask = 0;
    for (size_t zone = 0; zone < TOF_ZONE_COUNT; ++zone) {
        if (tof_status_is_valid(frame->status[zone])) {
            valid_mask |= UINT64_C(1) << zone;
        }
    }
    return valid_mask;
}

static void send_frame(lp_state_t *state, const tof_frame_t *frame,
                       uint64_t valid_mask)
{
    uint8_t packet[LORA_PROTOCOL_FRAME_SIZE];
    const lora_uplink_t uplink = uplink_of(state);
    const size_t length = lora_protocol_encode_frame(
        packet, sizeof(packet), &uplink, valid_mask, frame->distance_mm);
    send_encoded(packet, length, "frame");
    ESP_LOGI(TAG, "frame=%" PRIu32 " valid=%u/64", frame->frame_number,
             (unsigned)__builtin_popcountll(valid_mask));
}

/* A frame the sensor hands over with every zone invalid is a failed
 * measurement in practice: ranging restarts every wake, and the first frame
 * after start_ranging is often not settled yet. Retry those rather than
 * transmitting an empty matrix. */
static void measure_once(lp_state_t *state)
{
    tof_frame_t frame;
    uint64_t valid_mask = 0;
    bool have_frame = false;

    for (unsigned attempt = 1; attempt <= (unsigned)CONFIG_APP_MEASURE_ATTEMPTS;
         ++attempt) {
        const esp_err_t err = tof_get_frame(
            &state->sensor, &frame, pdMS_TO_TICKS(TOF_FRAME_TIMEOUT_MS));
        if (err != ESP_OK) {
            state->attempts_used++;
            ESP_LOGW(TAG, "tof_get_frame attempt %u/%d failed: %s", attempt,
                     CONFIG_APP_MEASURE_ATTEMPTS, esp_err_to_name(err));
            continue;
        }

        have_frame = true;
        valid_mask = frame_valid_mask(&frame);
        if (valid_mask != 0) {
            send_frame(state, &frame, valid_mask);
            return;
        }

        state->attempts_used++;
        ESP_LOGW(TAG, "attempt %u/%d returned no valid zones", attempt,
                 CONFIG_APP_MEASURE_ATTEMPTS);
    }

    if (have_frame) {
        /* Report the empty matrix rather than hiding it; A sees valid_mask=0. */
        send_frame(state, &frame, valid_mask);
    } else {
        send_status(state, LORA_STATUS_TOF_READ_FAILED);
    }
}

/* Order matters: ranging must stop before the ULD accepts a power-mode change,
 * and the SLEEPING packet has to go out before the radio itself sleeps.
 * With sleep disabled (sleep_s = 0) the sensor and the radio stay awake,
 * because putting them under for zero milliseconds only costs time and leaves
 * the radio asleep for the next cycle. */
/* Ranging burns ~100 mA, and nothing needs the sensor once the frames are out,
 * so this runs straight after the measurements rather than at the end of the
 * wake window. Holding ranging through the whole window was both wasteful and
 * where tof_stop_ranging kept failing. */
static void stop_sensor(lp_state_t *state, bool will_sleep)
{
    if (state->ranging) {
        /* If the ULD did not accept the stop, it still believes it is ranging
         * and will refuse every later power-mode change. Drop the handle so the
         * next cycle rebuilds it instead of looping on ESP_ERR_INVALID_STATE. */
        if (tof_stop_ranging(&state->sensor) != ESP_OK) {
            ESP_LOGW(TAG, "tof_stop_ranging failed, forcing a reinit");
            tof_deinit(&state->sensor);
            state->sensor_ready = false;
        }
        state->ranging = false;
    }
    if (will_sleep && state->sensor_ready &&
        tof_sleep(&state->sensor) != ESP_OK) {
        ESP_LOGW(TAG, "tof_sleep failed");
        send_status(state, LORA_STATUS_TOF_SLEEP_FAILED);
    }
}

static void shut_down_for_sleep(lp_state_t *state, uint32_t planned_sleep_ms,
                                bool will_sleep)
{
    send_sleeping(state, planned_sleep_ms);

    state->telemetry.total_awake_ms += awake_elapsed_ms(state);

    if (will_sleep && lora_radio_sleep() != ESP_OK) {
        ESP_LOGW(TAG, "lora_radio_sleep failed");
    }
}

static void enter_light_sleep(lp_state_t *state, uint32_t sleep_ms)
{
    if (sleep_ms == 0) {
        state->telemetry.last_sleep_ms = 0;
        return;
    }

    /* Light sleep cuts the UART mid-character otherwise. */
    fflush(stdout);
    esp_rom_output_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);

    esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);
    const int64_t before = now_us();
    esp_light_sleep_start();
    const int64_t after = now_us();

    /* esp_timer keeps counting across light sleep, so this is the real figure
     * rather than the requested one. */
    const uint32_t actual_ms = (uint32_t)((after - before) / 1000);
    state->telemetry.last_sleep_ms = actual_ms;
    state->telemetry.total_sleep_ms += actual_ms;
}

static void run_cycle(lp_state_t *state)
{
    /* A new config takes effect from this cycle on, which is what the ACK
     * promised when it was queued during the previous wake window. */
    if (state->has_pending) {
        state->applied = state->pending;
        state->has_pending = false;
        ESP_LOGI(TAG, "applied revision %u: sleep=%us wake=%us measure=%u",
                 state->applied.revision, state->applied.sleep_s,
                 state->applied.wake_s, state->applied.measure_count);
    }

    state->wake_start_us = now_us();
    state->attempts_used = 0;
    state->telemetry.wake_count++;
    state->telemetry.vbat_mv = vbat_read_mv();

    /* Both nodes wake on the same schedule, so separate their first packet. */
    const uint32_t slot_ms = lora_protocol_tx_slot_offset_ms(state->node_id);
    if (slot_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(slot_ms));
    }

    send_wake(state);
    service_config(state, EARLY_CONFIG_WINDOW_MS);

    const uint32_t sleep_ms = (uint32_t)state->applied.sleep_s * 1000U;
    const bool will_sleep = sleep_ms > 0;

    if (prepare_sensor(state)) {
        for (unsigned i = 0; i < state->applied.measure_count; ++i) {
            measure_once(state);
        }
        stop_sensor(state, will_sleep);
    }

    /* Whatever is left of the window stays open for A. */
    const uint32_t remaining = window_remaining_ms(state);
    if (remaining == 0) {
        ESP_LOGW(TAG, "wake window overran: %" PRIu32 " ms used of %u s",
                 awake_elapsed_ms(state), state->applied.wake_s);
    } else {
        service_config(state, remaining);
    }

    /* Sleeping and waking must stay paired: anything put under here has to be
     * brought back before the next cycle transmits. */
    shut_down_for_sleep(state, sleep_ms, will_sleep);
    enter_light_sleep(state, sleep_ms);

    if (will_sleep && lora_radio_wakeup() != ESP_OK) {
        ESP_LOGW(TAG, "lora_radio_wakeup failed, doing a full init");
        if (lora_radio_init() != ESP_OK) {
            ESP_LOGE(TAG, "lora_radio_init failed after sleep");
            send_status(state, LORA_STATUS_RADIO_SLEEP_FAILED);
        }
    }
}

void run_lowpower_sender(uint8_t node_id)
{
    lp_state_t state = {
        .node_id = node_id,
        .applied = {
            .revision = 0,
            .sleep_s = CONFIG_APP_DEFAULT_SLEEP_S,
            .wake_s = CONFIG_APP_DEFAULT_WAKE_S,
            .measure_count = CONFIG_APP_DEFAULT_MEASURE_COUNT,
        },
    };

    ESP_LOGI(TAG,
             "B-%u low-power sender: %lu Hz, %d dBm, sleep=%us wake=%us x%u",
             node_id, LORA_RADIO_FREQUENCY_HZ, LORA_RADIO_TX_POWER_DBM,
             state.applied.sleep_s, state.applied.wake_s,
             state.applied.measure_count);

    vbat_init();
    state.telemetry.vbat_mv = vbat_read_mv();
    ESP_LOGI(TAG, "battery: %u mV", state.telemetry.vbat_mv);

    ESP_ERROR_CHECK(lora_radio_init());

    /* The first transmit is the largest current draw the board has made so far,
     * and doing it the instant the radio comes up - while the supply is still
     * settling after boot - can brown out a battery that is only partly
     * charged. The Arduino sketch that ran these boards from battery finished
     * its whole setup first. */
    vTaskDelay(pdMS_TO_TICKS(BOOT_SETTLE_MS));

    state.wake_start_us = now_us();
    send_status(&state, LORA_STATUS_BOOT);

    while (true) {
        run_cycle(&state);
    }
}
