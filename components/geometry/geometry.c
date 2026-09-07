#include "geometry.h"

#include <math.h>

#define DEG_TO_RAD (0.01745329251994329577f)

geometry_point_t geometry_project_zone(uint16_t distance_mm, unsigned row,
                                       unsigned col,
                                       const geometry_mount_t *mount)
{
    const geometry_mount_t default_mount = {
        .horizontal_fov_deg = 45.0f,
        .vertical_fov_deg = 45.0f,
        .pitch_deg = 0.0f,
        .roll_deg = 0.0f,
    };
    if (!mount) {
        mount = &default_mount;
    }

    const float yaw = (((float)col + 0.5f) / 8.0f - 0.5f) *
                      mount->horizontal_fov_deg * DEG_TO_RAD;
    const float pitch = (((float)row + 0.5f) / 8.0f - 0.5f) *
                        mount->vertical_fov_deg * DEG_TO_RAD +
                        mount->pitch_deg * DEG_TO_RAD;
    const float roll = mount->roll_deg * DEG_TO_RAD;
    const float distance = (float)distance_mm;

    float x = distance * sinf(yaw) * cosf(pitch);
    float y = distance * sinf(pitch);
    float z = distance * cosf(yaw) * cosf(pitch);

    geometry_point_t point = {
        .x_mm = x * cosf(roll) - y * sinf(roll),
        .y_mm = x * sinf(roll) + y * cosf(roll),
        .z_mm = z,
    };
    return point;
}

int16_t height_from_baseline(uint16_t baseline_mm, uint16_t distance_mm,
                             bool valid)
{
    if (!valid) {
        return 0;
    }
    int32_t height = (int32_t)baseline_mm - (int32_t)distance_mm;
    if (height > INT16_MAX) {
        return INT16_MAX;
    }
    if (height < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)height;
}

size_t height_estimate_frame(const uint16_t baseline_mm[GEOMETRY_ZONE_COUNT],
                             const uint16_t distance_mm[GEOMETRY_ZONE_COUNT],
                             const uint8_t status[GEOMETRY_ZONE_COUNT],
                             int16_t height_mm[GEOMETRY_ZONE_COUNT])
{
    size_t valid_count = 0;
    for (size_t zone = 0; zone < GEOMETRY_ZONE_COUNT; ++zone) {
        const bool valid = status[zone] == 5 || status[zone] == 9;
        height_mm[zone] = height_from_baseline(baseline_mm[zone],
                                               distance_mm[zone], valid);
        valid_count += valid ? 1U : 0U;
    }
    return valid_count;
}
