#pragma once

#include <stdint.h>

/* Low-power B-node cycle: wake, serve config, measure, transmit, then put the
 * sensor, the radio and the CPU to sleep. Never returns. */
void run_lowpower_sender(uint8_t node_id);
