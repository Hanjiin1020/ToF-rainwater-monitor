#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "tof_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_MODE_CSV,
    APP_MODE_REPEAT,
    APP_MODE_SPECULAR,
    APP_MODE_WATER,
    APP_MODE_HEIGHT_DEMO,
} app_mode_t;

typedef struct {
    app_mode_t mode;
    uint32_t repeat_frame_count;
    uint16_t installation_height_mm;
    uint32_t output_interval_ms;
} single_sensor_test_config_t;

esp_err_t single_sensor_test_run(tof_sensor_t *sensor,
                                 const single_sensor_test_config_t *config);

#ifdef __cplusplus
}
#endif
