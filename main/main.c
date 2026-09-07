#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "bridge.h"
#include "i2c_diag.h"
#include "sender_lowpower.h"
#include "single_sensor_test.h"
#include "telemetry.h"
#include "tof_driver.h"

/* The i2cdiag role links only run_i2c_diag(); none of the code below applies. */
#if !defined(APP_NODE_ROLE_I2CDIAG)

static const char *TAG = "tof_lora";

static void board_enable_vext(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = UINT64_C(1) << BOARD_VEXT_CONTROL_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_set_level(BOARD_VEXT_CONTROL_GPIO,
                                   BOARD_VEXT_ENABLED_LEVEL));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Vext enabled on GPIO%d", BOARD_VEXT_CONTROL_GPIO);
}

#if defined(APP_NODE_ROLE_SENDER)

/* The whole sender lives in sender_lowpower.c; run_sender() only names it. */
static void run_sender(void)
{
    run_lowpower_sender(APP_NODE_ID);
}

#elif defined(APP_NODE_ROLE_RECEIVER)

/* Role A lives in bridge.c; run_receiver() only names it. */
static void run_receiver(void)
{
    run_bridge();
}

#else

static app_mode_t selected_mode(void)
{
#if CONFIG_APP_MODE_SST_REPEAT
    return APP_MODE_REPEAT;
#elif CONFIG_APP_MODE_SST_SPECULAR
    return APP_MODE_SPECULAR;
#elif CONFIG_APP_MODE_SST_WATER
    return APP_MODE_WATER;
#elif CONFIG_APP_MODE_HEIGHT_DEMO
    return APP_MODE_HEIGHT_DEMO;
#else
    return APP_MODE_CSV;
#endif
}

static void run_height_demo(tof_sensor_t *sensor)
{
    uint16_t baseline[TOF_ZONE_COUNT];
    for (size_t i = 0; i < TOF_ZONE_COUNT; ++i) {
        baseline[i] = HEIGHT_DEMO_PLACEHOLDER_BASELINE_MM;
    }

    ESP_LOGW(TAG, "HEIGHT_DEMO uses a placeholder baseline of %u mm",
             HEIGHT_DEMO_PLACEHOLDER_BASELINE_MM);
    for (unsigned frame_index = 0; frame_index < 10; ++frame_index) {
        tof_frame_t frame;
        ESP_ERROR_CHECK(tof_get_frame(sensor, &frame, portMAX_DELAY));
        telemetry_print_height_json(&frame, baseline);
    }
}

/* Only the diagnostic role uses this; the low-power sender does its own
 * retry and recovery. */
static esp_err_t init_tof_with_retries(tof_sensor_t *sensor)
{
    const tof_config_t config = app_default_tof_config();
    esp_err_t err = ESP_FAIL;
    for (unsigned attempt = 1; attempt <= TOF_INIT_RETRY_COUNT; ++attempt) {
        err = tof_init(sensor, &config);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "VL53L8CX found at 0x%02x on SDA=%d SCL=%d",
                     TOF_DEFAULT_ADDRESS_7BIT, TOF_PIN_SDA, TOF_PIN_SCL);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "ToF init attempt %u/%u failed: %s", attempt,
                 TOF_INIT_RETRY_COUNT, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(TOF_INIT_RETRY_DELAY_MS));
    }
    return err;
}

static void run_diagnostic(void)
{
    ESP_LOGI(TAG, "role=diagnostic, target=esp32s3");
    ESP_LOGI(TAG, "ToF wiring: SDA=%d SCL=%d address=0x%02x",
             TOF_PIN_SDA, TOF_PIN_SCL, TOF_DEFAULT_ADDRESS_7BIT);

    tof_sensor_t sensor = {0};
    const esp_err_t err = init_tof_with_retries(&sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tof_init failed after retries: %s",
                 esp_err_to_name(err));
        return;
    }
    ESP_ERROR_CHECK(tof_start_ranging(&sensor));

    const app_mode_t mode = selected_mode();
    if (mode == APP_MODE_HEIGHT_DEMO) {
        run_height_demo(&sensor);
    } else {
        const single_sensor_test_config_t test_config = {
            .mode = mode,
            .repeat_frame_count = CONFIG_APP_REPEAT_FRAME_COUNT,
            .installation_height_mm = CONFIG_APP_WATER_INSTALL_HEIGHT_MM,
            .output_interval_ms = CONFIG_APP_OUTPUT_INTERVAL_MS,
        };
        single_sensor_test_run(&sensor, &test_config);
    }

    tof_stop_ranging(&sensor);
    tof_deinit(&sensor);
}

#endif

#endif /* !APP_NODE_ROLE_I2CDIAG */

void app_main(void)
{
#if defined(APP_NODE_ROLE_RECEIVER)
    run_receiver();
#elif defined(APP_NODE_ROLE_SENDER)
    board_enable_vext();
    run_sender();
#elif defined(APP_NODE_ROLE_I2CDIAG)
    /* run_i2c_diag() drives Vext itself so both states can be compared. */
    run_i2c_diag();
#else
    board_enable_vext();
    run_diagnostic();
#endif
}
