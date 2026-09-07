#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GEOMETRY_ZONE_COUNT 64U

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
} geometry_point_t;

typedef struct {
    float horizontal_fov_deg;
    float vertical_fov_deg;
    float pitch_deg;
    float roll_deg;
} geometry_mount_t;

geometry_point_t geometry_project_zone(uint16_t distance_mm, unsigned row,
                                       unsigned col,
                                       const geometry_mount_t *mount);
int16_t height_from_baseline(uint16_t baseline_mm, uint16_t distance_mm,
                             bool valid);
size_t height_estimate_frame(const uint16_t baseline_mm[GEOMETRY_ZONE_COUNT],
                             const uint16_t distance_mm[GEOMETRY_ZONE_COUNT],
                             const uint8_t status[GEOMETRY_ZONE_COUNT],
                             int16_t height_mm[GEOMETRY_ZONE_COUNT]);

#ifdef __cplusplus
}
#endif
