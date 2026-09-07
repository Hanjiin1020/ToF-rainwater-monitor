#include "single_sensor_test.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "telemetry.h"

static const char *TAG = "SST";

static esp_err_t run_csv(tof_sensor_t *sensor, uint32_t output_interval_ms)
{
    ESP_LOGI(TAG, "8x8 logger start: one frame every %" PRIu32 " ms",
             output_interval_ms);
    while (true) {
        tof_frame_t frame;
        esp_err_t err = tof_get_frame(sensor, &frame, portMAX_DELAY);
        if (err != ESP_OK) {
            return err;
        }
        telemetry_print_distance_matrix(&frame, 0);
        vTaskDelay(pdMS_TO_TICKS(output_interval_ms));
    }
}

static esp_err_t run_repeatability(tof_sensor_t *sensor, uint32_t frame_count)
{
    double sum[TOF_ZONE_COUNT] = {0};
    double sum_sq[TOF_ZONE_COUNT] = {0};
    uint32_t count[TOF_ZONE_COUNT] = {0};
    ESP_LOGI(TAG, "repeatability start: %" PRIu32 " frames", frame_count);

    for (uint32_t n = 0; n < frame_count; ++n) {
        tof_frame_t frame;
        esp_err_t err = tof_get_frame(sensor, &frame, portMAX_DELAY);
        if (err != ESP_OK) {
            return err;
        }
        for (size_t zone = 0; zone < TOF_ZONE_COUNT; ++zone) {
            if (tof_status_is_valid(frame.status[zone])) {
                const double value = frame.distance_mm[zone];
                sum[zone] += value;
                sum_sq[zone] += value * value;
                ++count[zone];
            }
        }
    }

    puts("REPEAT,zone,valid_count,mean_mm,stddev_mm");
    for (size_t zone = 0; zone < TOF_ZONE_COUNT; ++zone) {
        const double mean = count[zone] ? sum[zone] / count[zone] : 0.0;
        double variance = count[zone] ? sum_sq[zone] / count[zone] - mean * mean : 0.0;
        if (variance < 0.0) {
            variance = 0.0;
        }
        printf("REPEAT,%u,%" PRIu32 ",%.2f,%.2f\n", (unsigned)zone,
               count[zone], mean, sqrt(variance));
    }
    return ESP_OK;
}

static esp_err_t run_specular(tof_sensor_t *sensor)
{
    ESP_LOGI(TAG, "specular response logger start");
    puts("SPECULAR,t_us,frame,center_signal,outer_signal,center_outer_ratio");
    while (true) {
        tof_frame_t frame;
        esp_err_t err = tof_get_frame(sensor, &frame, portMAX_DELAY);
        if (err != ESP_OK) {
            return err;
        }
        uint64_t center_sum = 0, outer_sum = 0;
        unsigned center_count = 0, outer_count = 0;
        for (size_t zone = 0; zone < TOF_ZONE_COUNT; ++zone) {
            const unsigned row = zone / TOF_COLS;
            const unsigned col = zone % TOF_COLS;
            if (!tof_status_is_valid(frame.status[zone])) {
                continue;
            }
            if (row >= 2 && row <= 5 && col >= 2 && col <= 5) {
                center_sum += frame.signal_per_spad[zone];
                ++center_count;
            } else {
                outer_sum += frame.signal_per_spad[zone];
                ++outer_count;
            }
        }
        const double center = center_count ? (double)center_sum / center_count : 0.0;
        const double outer = outer_count ? (double)outer_sum / outer_count : 0.0;
        printf("SPECULAR,%" PRIu32 ",%" PRIu32 ",%.2f,%.2f,%.4f\n",
               frame.timestamp_us, frame.frame_number, center, outer,
               outer > 0.0 ? center / outer : 0.0);
    }
}

static esp_err_t run_water(tof_sensor_t *sensor, uint16_t installation_height_mm)
{
    static const uint8_t center_zones[] = {
        18, 19, 20, 21, 26, 27, 28, 29, 34, 35, 36, 37,
    };
    ESP_LOGI(TAG, "water logger start: installation height=%u mm",
             installation_height_mm);
    puts("WATER,t_us,frame,distance_mm,level_mm,valid_center_zones");
    while (true) {
        tof_frame_t frame;
        esp_err_t err = tof_get_frame(sensor, &frame, portMAX_DELAY);
        if (err != ESP_OK) {
            return err;
        }
        uint32_t strongest_signal = 0;
        uint16_t selected_distance = 0;
        unsigned valid_count = 0;
        for (size_t i = 0; i < sizeof(center_zones); ++i) {
            const uint8_t zone = center_zones[i];
            if (tof_status_is_valid(frame.status[zone])) {
                ++valid_count;
                if (frame.signal_per_spad[zone] > strongest_signal) {
                    strongest_signal = frame.signal_per_spad[zone];
                    selected_distance = frame.distance_mm[zone];
                }
            }
        }
        const int level_mm = valid_count ?
            (int)installation_height_mm - (int)selected_distance : 0;
        printf("WATER,%" PRIu32 ",%" PRIu32 ",%u,%d,%u\n",
               frame.timestamp_us, frame.frame_number, selected_distance,
               level_mm, valid_count);
    }
}

esp_err_t single_sensor_test_run(tof_sensor_t *sensor,
                                 const single_sensor_test_config_t *config)
{
    if (!sensor || !config) {
        return ESP_ERR_INVALID_ARG;
    }
    switch (config->mode) {
    case APP_MODE_CSV:
        return run_csv(sensor, config->output_interval_ms);
    case APP_MODE_REPEAT:
        return run_repeatability(sensor, config->repeat_frame_count);
    case APP_MODE_SPECULAR:
        return run_specular(sensor);
    case APP_MODE_WATER:
        return run_water(sensor, config->installation_height_mm);
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}
