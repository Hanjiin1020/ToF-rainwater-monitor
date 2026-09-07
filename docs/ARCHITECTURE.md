# Firmware architecture

- `main`: selects the Kconfig run mode and owns application lifecycle.
- `tof_driver`: owns the I2C bus and VL53L8CX ULD state, and emits normalized
  8x8 frames.
- `single_sensor_test`: implements CSV, repeatability, specular-response, and
  water-level experiment loops.
- `geometry`: pure-C coordinate projection and baseline height estimation.
- `telemetry`: serial CSV and JSON formatting.

LoRa is deliberately absent until the exact radio, pin map, region, and network
mode are confirmed. See `SPEC.md` section 9.
