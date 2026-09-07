#include <assert.h>
#include <stdbool.h>

#include "geometry.h"

int main(void)
{
    assert(height_from_baseline(1000, 750, true) == 250);
    assert(height_from_baseline(1000, 1100, true) == -100);
    assert(height_from_baseline(1000, 750, false) == 0);
    assert(height_from_baseline(65535, 0, true) == 32767);
    return 0;
}
