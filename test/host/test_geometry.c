#include <assert.h>
#include <math.h>

#include "geometry.h"

int main(void)
{
    const geometry_mount_t zero_fov = {
        .horizontal_fov_deg = 0.0f,
        .vertical_fov_deg = 0.0f,
        .pitch_deg = 0.0f,
        .roll_deg = 0.0f,
    };
    geometry_point_t point = geometry_project_zone(1000, 3, 3, &zero_fov);
    assert(fabsf(point.x_mm) < 0.001f);
    assert(fabsf(point.y_mm) < 0.001f);
    assert(fabsf(point.z_mm - 1000.0f) < 0.001f);

    const geometry_mount_t pitched = {
        .horizontal_fov_deg = 0.0f,
        .vertical_fov_deg = 0.0f,
        .pitch_deg = 30.0f,
        .roll_deg = 0.0f,
    };
    point = geometry_project_zone(1000, 0, 0, &pitched);
    assert(fabsf(point.y_mm - 500.0f) < 0.1f);
    assert(fabsf(point.z_mm - 866.0254f) < 0.1f);
    return 0;
}
