#pragma once

/* Role A: receives from every B node, forwards what it sees to the Mac as
 * JSON lines, and pushes queued configuration down to the nodes. Never
 * returns. */
void run_bridge(void);
