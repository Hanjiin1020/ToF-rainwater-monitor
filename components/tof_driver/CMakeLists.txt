idf_component_register(
    SRCS "tof_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer rjrp44__vl53l8cx
)
