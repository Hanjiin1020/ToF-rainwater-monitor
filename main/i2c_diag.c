#include <inttypes.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "i2c_diag.h"
#include "telemetry.h"
#include "tof_driver.h"

static const char *TAG = "i2c_diag";

/* vl53l8cx_is_alive() accepts only this pair, read from page 0 registers 0/1. */
#define VL53L8CX_EXPECTED_DEVICE_ID 0xF0U
#define VL53L8CX_EXPECTED_REVISION_ID 0x0CU

#define PROBE_TIMEOUT_MS 50
#define XFER_TIMEOUT_MS 200
#define VEXT_SETTLE_MS 200

/* Hold test: the slowest clock gives marginal joints the most settling time,
 * and retries never stop. */
#define HOLD_I2C_SPEED_HZ 50000U
#define HOLD_RETRY_DELAY_MS 200
#define HOLD_FRAME_TIMEOUT_MS 1000
#define HOLD_MAX_FRAME_ERRORS 5
#define LINE_SETTLE_MS 5U
#define LINE_SAMPLES 12U
#define LINE_SAMPLE_GAP_MS 3U

static const uint32_t k_speeds_hz[] = {50000U, 100000U, 400000U, 1000000U};

/* Idle bus levels tell us whether the carrier is powered and the lines are
 * free, which is otherwise a multimeter measurement. Must run before the I2C
 * driver claims the pins. */
static void report_line_levels(void)
{
    const uint64_t mask = ((uint64_t)1 << TOF_PIN_SDA) | ((uint64_t)1 << TOF_PIN_SCL);

    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    vTaskDelay(pdMS_TO_TICKS(20));
    const int sda_ext = gpio_get_level(TOF_PIN_SDA);
    const int scl_ext = gpio_get_level(TOF_PIN_SCL);

    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));
    vTaskDelay(pdMS_TO_TICKS(20));
    const int sda_int = gpio_get_level(TOF_PIN_SDA);
    const int scl_int = gpio_get_level(TOF_PIN_SCL);

    ESP_LOGI(TAG, "no internal pull-up : SDA=%d SCL=%d (1 = carrier pull-ups alive)",
             sda_ext, scl_ext);
    ESP_LOGI(TAG, "internal pull-up on : SDA=%d SCL=%d (0 = line shorted to GND)",
             sda_int, scl_int);

    if (sda_ext == 0 && scl_ext == 0) {
        ESP_LOGW(TAG, "both lines low without internal pull-up: carrier looks unpowered");
    }
    if (sda_int == 0 || scl_int == 0) {
        ESP_LOGE(TAG, "line stays low against the internal pull-up: short or stuck device");
    }
}

static void set_vext(bool enabled)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (uint64_t)1 << BOARD_VEXT_CONTROL_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_VEXT_CONTROL_GPIO,
                                   enabled ? BOARD_VEXT_ENABLED_LEVEL
                                           : !BOARD_VEXT_ENABLED_LEVEL));
    vTaskDelay(pdMS_TO_TICKS(VEXT_SETTLE_MS));
}

static esp_err_t open_bus(i2c_master_bus_handle_t *bus)
{
    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TOF_I2C_PORT,
        .scl_io_num = TOF_PIN_SCL,
        .sda_io_num = TOF_PIN_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_config, bus);
}

static unsigned scan_addresses(i2c_master_bus_handle_t bus)
{
    unsigned found = 0;
    for (uint16_t address = 0x08; address <= 0x77; ++address) {
        if (i2c_master_probe(bus, address, PROBE_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "  ACK at 0x%02X%s", address,
                     address == TOF_DEFAULT_ADDRESS_7BIT ? "  <-- VL53L8CX" : "");
            ++found;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "  no device answered on the whole bus");
    }
    return found;
}

/* Repeats what vl53l8cx_is_alive() does, but keeps the transport error and the
 * returned ID separate so the two failure modes can be told apart. */
static void read_identity(i2c_master_bus_handle_t bus, uint32_t speed_hz)
{
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOF_DEFAULT_ADDRESS_7BIT,
        .scl_speed_hz = speed_hz,
    };
    i2c_master_dev_handle_t device = NULL;
    esp_err_t err = i2c_master_bus_add_device(bus, &device_config, &device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  %3" PRIu32 " kHz: add_device failed: %s",
                 speed_hz / 1000U, esp_err_to_name(err));
        return;
    }

    bool failed = true;

    const uint8_t select_page0[] = {0x7F, 0xFF, 0x00};
    err = i2c_master_transmit(device, select_page0, sizeof(select_page0),
                              XFER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "  %3" PRIu32 " kHz: no ACK from 0x%02X (%s)",
                 speed_hz / 1000U, TOF_DEFAULT_ADDRESS_7BIT, esp_err_to_name(err));
        goto done;
    }

    const uint8_t reg_device_id[] = {0x00, 0x00};
    const uint8_t reg_revision_id[] = {0x00, 0x01};
    uint8_t device_id = 0;
    uint8_t revision_id = 0;
    esp_err_t err_id = i2c_master_transmit_receive(device, reg_device_id,
                                                   sizeof(reg_device_id),
                                                   &device_id, 1, XFER_TIMEOUT_MS);
    esp_err_t err_rev = i2c_master_transmit_receive(device, reg_revision_id,
                                                    sizeof(reg_revision_id),
                                                    &revision_id, 1, XFER_TIMEOUT_MS);
    if (err_id != ESP_OK || err_rev != ESP_OK) {
        ESP_LOGE(TAG, "  %3" PRIu32 " kHz: ACK but register read failed (%s / %s)",
                 speed_hz / 1000U, esp_err_to_name(err_id), esp_err_to_name(err_rev));
        goto done;
    }

    const bool alive = device_id == VL53L8CX_EXPECTED_DEVICE_ID &&
                       revision_id == VL53L8CX_EXPECTED_REVISION_ID;
    ESP_LOGI(TAG, "  %3" PRIu32 " kHz: device_id=0x%02X revision_id=0x%02X -> %s",
             speed_hz / 1000U, device_id, revision_id,
             alive ? "ALIVE" : "wrong ID (expected 0xF0 / 0x0C)");
    failed = false;

done:
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(device));
    /* A half-finished transfer can leave SDA held low, which would make every
     * later speed fail for the wrong reason. */
    if (failed) {
        i2c_master_bus_reset(bus);
    }
}

/* Safe to call only while no I2C bus owns the pins.
 *
 * The pins have just been released by the I2C driver, so they need time to
 * settle before they mean anything - reading immediately after gpio_config()
 * reports whatever charge was left on the line and looks like a flapping
 * connection. Several samples are taken so a genuinely unstable joint can be
 * told apart from a steady one. */
static void log_line_levels(const char *prefix)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = ((uint64_t)1 << TOF_PIN_SDA) | ((uint64_t)1 << TOF_PIN_SCL),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&cfg) != ESP_OK) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(LINE_SETTLE_MS));

    unsigned sda_high = 0, scl_high = 0;
    for (unsigned i = 0; i < LINE_SAMPLES; ++i) {
        sda_high += (unsigned)gpio_get_level(TOF_PIN_SDA);
        scl_high += (unsigned)gpio_get_level(TOF_PIN_SCL);
        vTaskDelay(pdMS_TO_TICKS(LINE_SAMPLE_GAP_MS));
    }

    const bool sda_steady = sda_high == 0 || sda_high == LINE_SAMPLES;
    const bool scl_steady = scl_high == 0 || scl_high == LINE_SAMPLES;
    const char *verdict;
    if (!sda_steady || !scl_steady) {
        verdict = "  (unstable: a joint is making and breaking)";
    } else if (sda_high && scl_high) {
        verdict = "  (both wires connected and carrier powered)";
    } else if (!sda_high && !scl_high) {
        verdict = "  (both low: VIN or GND is open)";
    } else {
        verdict = "  (one wire open)";
    }
    ESP_LOGW(TAG, "%s SDA=%u/%u SCL=%u/%u%s", prefix, sda_high, LINE_SAMPLES,
             scl_high, LINE_SAMPLES, verdict);
}

static tof_config_t hold_tof_config(void)
{
    tof_config_t config = app_default_tof_config();
    config.i2c_speed_hz = HOLD_I2C_SPEED_HZ;
    return config;
}

/* Retries forever instead of the normal three attempts, so an intermittent
 * harness can be held by hand until it catches. */
static void run_hold_test(void)
{
    const tof_config_t config = hold_tof_config();
    ESP_LOGI(TAG, "hold test at %u kHz: press VIN, GND, SDA and SCL and wait",
             (unsigned)(HOLD_I2C_SPEED_HZ / 1000U));

    while (true) {
        tof_sensor_t sensor = {0};
        unsigned attempts = 0;
        while (tof_init(&sensor, &config) != ESP_OK) {
            ++attempts;
            if (attempts % 10 == 0) {
                log_line_levels("not detected yet:");
            }
            vTaskDelay(pdMS_TO_TICKS(HOLD_RETRY_DELAY_MS));
        }
        ESP_LOGI(TAG, "*** SENSOR DETECTED (after %u retries) ***", attempts);

        esp_err_t err = tof_start_ranging(&sensor);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "start_ranging failed: %s", esp_err_to_name(err));
            tof_deinit(&sensor);
            continue;
        }

        unsigned consecutive_errors = 0;
        while (consecutive_errors < HOLD_MAX_FRAME_ERRORS) {
            tof_frame_t frame;
            err = tof_get_frame(&sensor, &frame,
                                pdMS_TO_TICKS(HOLD_FRAME_TIMEOUT_MS));
            if (err != ESP_OK) {
                ++consecutive_errors;
                ESP_LOGW(TAG, "frame read failed (%u/%u): %s", consecutive_errors,
                         HOLD_MAX_FRAME_ERRORS, esp_err_to_name(err));
                continue;
            }
            consecutive_errors = 0;
            telemetry_print_distance_matrix(&frame, 0);
        }

        ESP_LOGE(TAG, "lost contact, going back to detection");
        tof_stop_ranging(&sensor);
        tof_deinit(&sensor);
    }
}

static void run_pass(const char *label)
{
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = open_bus(&bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] i2c_new_master_bus failed: %s", label,
                 esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "[%s] scanning 0x08..0x77", label);
    scan_addresses(bus);

    ESP_LOGI(TAG, "[%s] reading identity at 0x%02X", label,
             TOF_DEFAULT_ADDRESS_7BIT);
    for (size_t i = 0; i < sizeof(k_speeds_hz) / sizeof(k_speeds_hz[0]); ++i) {
        read_identity(bus, k_speeds_hz[i]);
    }

    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
}

void run_i2c_diag(void)
{
    ESP_LOGI(TAG, "VL53L8CX I2C diagnostic: SDA=GPIO%d SCL=GPIO%d addr=0x%02X",
             TOF_PIN_SDA, TOF_PIN_SCL, TOF_DEFAULT_ADDRESS_7BIT);

    ESP_LOGI(TAG, "--- idle line levels ---");
    report_line_levels();

    ESP_LOGI(TAG, "--- Vext OFF ---");
    set_vext(false);
    run_pass("Vext OFF");

    ESP_LOGI(TAG, "--- Vext ON ---");
    set_vext(true);
    run_pass("Vext ON");

    ESP_LOGI(TAG, "--- hold test: 8x8 frames for as long as contact holds ---");
    run_hold_test();
}
