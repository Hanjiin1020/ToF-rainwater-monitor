#include "tof_driver.h"

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "vl53l8cx_api.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    VL53L8CX_Configuration device;
    VL53L8CX_ResultsData results;
    bool ranging;
} tof_impl_t;

static esp_err_t uld_error(uint8_t status)
{
    return status == 0 ? ESP_OK : ESP_FAIL;
}

bool tof_status_is_valid(uint8_t target_status)
{
    /* ST ULD: 5 and 9 are the normal ranging statuses. */
    return target_status == 5 || target_status == 9;
}

esp_err_t tof_init(tof_sensor_t *sensor, const tof_config_t *config)
{
    ESP_RETURN_ON_FALSE(sensor && config, ESP_ERR_INVALID_ARG,
                        "tof_driver", "null argument");
    ESP_RETURN_ON_FALSE(sensor->impl == NULL, ESP_ERR_INVALID_STATE,
                        "tof_driver", "already initialized");

    tof_impl_t *impl = calloc(1, sizeof(*impl));
    ESP_RETURN_ON_FALSE(impl, ESP_ERR_NO_MEM, "tof_driver", "allocation failed");

    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = config->i2c_port,
        .scl_io_num = config->scl_gpio,
        .sda_io_num = config->sda_gpio,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &impl->bus);
    if (err != ESP_OK) {
        free(impl);
        return err;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->address_7bit,
        .scl_speed_hz = config->i2c_speed_hz,
    };
    impl->device.platform.bus_config = bus_config;
    err = i2c_master_bus_add_device(impl->bus, &device_config,
                                    &impl->device.platform.handle);
    if (err != ESP_OK) {
        i2c_del_master_bus(impl->bus);
        free(impl);
        return err;
    }

    if (config->reset_gpio != GPIO_NUM_NC) {
        impl->device.platform.reset_gpio = config->reset_gpio;
        VL53L8CX_Reset_Sensor(&impl->device.platform);
    }

    uint8_t alive = 0;
    uint8_t status = vl53l8cx_is_alive(&impl->device, &alive);
    if (status != 0 || alive == 0) {
        err = ESP_ERR_NOT_FOUND;
        goto fail;
    }
    if (vl53l8cx_init(&impl->device) != 0 ||
        vl53l8cx_set_resolution(&impl->device, VL53L8CX_RESOLUTION_8X8) != 0 ||
        vl53l8cx_set_ranging_frequency_hz(&impl->device, config->ranging_hz) != 0) {
        err = ESP_FAIL;
        goto fail;
    }

    sensor->impl = impl;
    return ESP_OK;

fail:
    i2c_master_bus_rm_device(impl->device.platform.handle);
    i2c_del_master_bus(impl->bus);
    free(impl);
    return err;
}

esp_err_t tof_start_ranging(tof_sensor_t *sensor)
{
    ESP_RETURN_ON_FALSE(sensor && sensor->impl, ESP_ERR_INVALID_STATE,
                        "tof_driver", "not initialized");
    tof_impl_t *impl = sensor->impl;
    esp_err_t err = uld_error(vl53l8cx_start_ranging(&impl->device));
    impl->ranging = err == ESP_OK;
    return err;
}

esp_err_t tof_get_frame(tof_sensor_t *sensor, tof_frame_t *frame,
                        TickType_t timeout_ticks)
{
    ESP_RETURN_ON_FALSE(sensor && sensor->impl && frame, ESP_ERR_INVALID_ARG,
                        "tof_driver", "invalid argument");
    tof_impl_t *impl = sensor->impl;
    ESP_RETURN_ON_FALSE(impl->ranging, ESP_ERR_INVALID_STATE,
                        "tof_driver", "ranging not started");

    const TickType_t start = xTaskGetTickCount();
    uint8_t ready = 0;
    do {
        if (vl53l8cx_check_data_ready(&impl->device, &ready) != 0) {
            return ESP_FAIL;
        }
        if (ready) {
            break;
        }
        if (timeout_ticks != portMAX_DELAY &&
            (xTaskGetTickCount() - start) >= timeout_ticks) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    } while (true);

    if (vl53l8cx_get_ranging_data(&impl->device, &impl->results) != 0) {
        return ESP_FAIL;
    }

    memset(frame, 0, sizeof(*frame));
    frame->timestamp_us = (uint32_t)esp_timer_get_time();
    frame->frame_number = impl->device.streamcount;
    for (size_t zone = 0; zone < TOF_ZONE_COUNT; ++zone) {
        const size_t index = VL53L8CX_NB_TARGET_PER_ZONE * zone;
        frame->distance_mm[zone] = (uint16_t)impl->results.distance_mm[index];
        frame->status[zone] = impl->results.target_status[index];
        frame->signal_per_spad[zone] = impl->results.signal_per_spad[index];
    }
    return ESP_OK;
}

esp_err_t tof_stop_ranging(tof_sensor_t *sensor)
{
    ESP_RETURN_ON_FALSE(sensor && sensor->impl, ESP_ERR_INVALID_STATE,
                        "tof_driver", "not initialized");
    tof_impl_t *impl = sensor->impl;
    if (!impl->ranging) {
        return ESP_OK;
    }
    esp_err_t err = uld_error(vl53l8cx_stop_ranging(&impl->device));
    if (err == ESP_OK) {
        impl->ranging = false;
    }
    return err;
}

static esp_err_t set_power_mode(tof_sensor_t *sensor, uint8_t mode)
{
    ESP_RETURN_ON_FALSE(sensor && sensor->impl, ESP_ERR_INVALID_STATE,
                        "tof_driver", "not initialized");
    tof_impl_t *impl = sensor->impl;
    ESP_RETURN_ON_FALSE(!impl->ranging, ESP_ERR_INVALID_STATE, "tof_driver",
                        "stop ranging before changing power mode");
    return uld_error(vl53l8cx_set_power_mode(&impl->device, mode));
}

esp_err_t tof_sleep(tof_sensor_t *sensor)
{
    return set_power_mode(sensor, VL53L8CX_POWER_MODE_SLEEP);
}

esp_err_t tof_wakeup(tof_sensor_t *sensor)
{
    return set_power_mode(sensor, VL53L8CX_POWER_MODE_WAKEUP);
}

void tof_deinit(tof_sensor_t *sensor)
{
    if (!sensor || !sensor->impl) {
        return;
    }
    tof_impl_t *impl = sensor->impl;
    if (impl->ranging) {
        vl53l8cx_stop_ranging(&impl->device);
    }
    i2c_master_bus_rm_device(impl->device.platform.handle);
    i2c_del_master_bus(impl->bus);
    free(impl);
    sensor->impl = NULL;
}
