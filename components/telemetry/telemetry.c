#include "telemetry.h"

#include <inttypes.h>
#include <stdio.h>

#include "geometry.h"

void telemetry_print_csv(const tof_frame_t *frame, unsigned sensor_index)
{
    for (size_t zone = 0; zone < TOF_ZONE_COUNT; ++zone) {
        printf("CSV,%" PRIu32 ",%u,%" PRIu32 ",%u,%u,%u,%u,%u,%u,%" PRIu32 "\n",
               frame->timestamp_us, sensor_index, frame->frame_number,
               (unsigned)zone, (unsigned)(zone / TOF_COLS),
               (unsigned)(zone % TOF_COLS), frame->distance_mm[zone],
               frame->status[zone], tof_status_is_valid(frame->status[zone]),
               frame->signal_per_spad[zone]);
    }
}

void telemetry_print_distance_matrix(const tof_frame_t *frame,
                                     unsigned sensor_index)
{
    unsigned valid_count = 0;
    printf("\nTOF8X8 sensor=%u t_us=%" PRIu32 " frame=%" PRIu32 " distance_mm\n",
           sensor_index, frame->timestamp_us, frame->frame_number);
    for (size_t row = 0; row < TOF_ROWS; ++row) {
        printf("R%u |", (unsigned)row);
        for (size_t col = 0; col < TOF_COLS; ++col) {
            const size_t zone = row * TOF_COLS + col;
            if (tof_status_is_valid(frame->status[zone])) {
                printf(" %5u", frame->distance_mm[zone]);
                ++valid_count;
            } else {
                printf("  ----");
            }
        }
        putchar('\n');
    }
    printf("valid_zones=%u/%u\n", valid_count, TOF_ZONE_COUNT);
}

void telemetry_print_height_json(const tof_frame_t *frame,
                                 const uint16_t baseline_mm[TOF_ZONE_COUNT])
{
    int16_t height_mm[TOF_ZONE_COUNT];
    const size_t valid_count = height_estimate_frame(
        baseline_mm, frame->distance_mm, frame->status, height_mm);

    printf("{\"t_us\":%" PRIu32 ",\"frame\":%" PRIu32
           ",\"valid_zones\":%u,\"height_mm\":[",
           frame->timestamp_us, frame->frame_number, (unsigned)valid_count);
    for (size_t zone = 0; zone < TOF_ZONE_COUNT; ++zone) {
        printf("%s%d", zone == 0 ? "" : ",", height_mm[zone]);
    }
    printf("]}\n");
}
