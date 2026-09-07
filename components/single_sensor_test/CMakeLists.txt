idf_component_register(
    SRCS "single_sensor_test.c"
    INCLUDE_DIRS "include"
    REQUIRES tof_driver telemetry
)
