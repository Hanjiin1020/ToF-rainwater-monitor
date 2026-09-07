#pragma once

#include <stdint.h>

#include "tof_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

void telemetry_print_csv(const tof_frame_t *frame, unsigned sensor_index);
void telemetry_print_distance_matrix(const tof_frame_t *frame,
                                     unsigned sensor_index);
void telemetry_print_height_json(const tof_frame_t *frame,
                                 const uint16_t baseline_mm[TOF_ZONE_COUNT]);

#ifdef __cplusplus
}
#endif
