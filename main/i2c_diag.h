#pragma once

/* Standalone I2C bring-up diagnostic for the VL53L8CX carrier.
 * Selected with -DAPP_NODE_ROLE=i2cdiag; never linked into the other roles. */
void run_i2c_diag(void);
