idf_component_register(
    SRCS "lora_radio.c"
    INCLUDE_DIRS "include"
    REQUIRES ra01s esp_driver_spi
)
