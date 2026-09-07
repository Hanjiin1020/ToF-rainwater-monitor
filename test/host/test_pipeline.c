#include <assert.h>
#include <stddef.h>

#include "geometry.h"

int main(void)
{
    uint16_t baseline[GEOMETRY_ZONE_COUNT];
    uint16_t distance[GEOMETRY_ZONE_COUNT];
    uint8_t status[GEOMETRY_ZONE_COUNT];
    int16_t height[GEOMETRY_ZONE_COUNT];

    for (size_t zone = 0; zone < GEOMETRY_ZONE_COUNT; ++zone) {
        baseline[zone] = 1000;
        distance[zone] = (uint16_t)(900 + zone);
        status[zone] = 5;
    }
    status[7] = 255;

    const size_t valid = height_estimate_frame(baseline, distance, status, height);
    assert(valid == GEOMETRY_ZONE_COUNT - 1);
    assert(height[0] == 100);
    assert(height[7] == 0);
    assert(height[63] == 37);
    return 0;
}
