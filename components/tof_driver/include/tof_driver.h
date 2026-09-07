#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TOF_ROWS 8U
#define TOF_COLS 8U
#define TOF_ZONE_COUNT (TOF_ROWS * TOF_COLS)

typedef struct {
    i2c_port_num_t i2c_port;
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    gpio_num_t reset_gpio;
    uint8_t address_7bit;
    uint32_t i2c_speed_hz;
    uint8_t ranging_hz;
} tof_config_t;

typedef struct {
    uint32_t timestamp_us;
    uint32_t frame_number;
    uint16_t distance_mm[TOF_ZONE_COUNT];
    uint8_t status[TOF_ZONE_COUNT];
    uint32_t signal_per_spad[TOF_ZONE_COUNT];
} tof_frame_t;

typedef struct {
    void *impl;
} tof_sensor_t;

esp_err_t tof_init(tof_sensor_t *sensor, const tof_config_t *config);
esp_err_t tof_start_ranging(tof_sensor_t *sensor);
esp_err_t tof_get_frame(tof_sensor_t *sensor, tof_frame_t *frame,
                        TickType_t timeout_ticks);
esp_err_t tof_stop_ranging(tof_sensor_t *sensor);
void tof_deinit(tof_sensor_t *sensor);
bool tof_status_is_valid(uint8_t target_status);

/* ULD low-power mode. SLEEP keeps the firmware, configuration and calibration,
 * so a later tof_wakeup() only needs start_ranging again. The ULD refuses a
 * mode change while ranging, so tof_stop_ranging() must come first; both calls
 * return ESP_ERR_INVALID_STATE if ranging is still active. */
esp_err_t tof_sleep(tof_sensor_t *sensor);
esp_err_t tof_wakeup(tof_sensor_t *sensor);

#ifdef __cplusplus
}
#endif
