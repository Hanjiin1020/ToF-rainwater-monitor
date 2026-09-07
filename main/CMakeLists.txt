idf_component_register(
    SRCS "bridge.c" "i2c_diag.c" "main.c" "sender_lowpower.c"
    INCLUDE_DIRS "."
    REQUIRES
        tof_driver
        single_sensor_test
        geometry
        telemetry
        lora_protocol
        lora_radio
        esp_timer
        esp_driver_gpio
        esp_driver_i2c
        esp_hw_support
        esp_driver_uart
        esp_adc
)

set(APP_NODE_ROLE "diagnostic" CACHE STRING
    "Firmware role: diagnostic, sender, receiver, or i2cdiag")
set_property(CACHE APP_NODE_ROLE PROPERTY STRINGS
    diagnostic sender receiver i2cdiag)
set(APP_NODE_ID "1" CACHE STRING "Sender node ID (B-1=1, B-2=2)")

string(TOLOWER "${APP_NODE_ROLE}" APP_NODE_ROLE_LOWER)
if(APP_NODE_ROLE_LOWER STREQUAL "sender")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        APP_NODE_ROLE_SENDER=1
        APP_NODE_ID=${APP_NODE_ID}
    )
elseif(APP_NODE_ROLE_LOWER STREQUAL "receiver")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        APP_NODE_ROLE_RECEIVER=1
    )
elseif(APP_NODE_ROLE_LOWER STREQUAL "diagnostic")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        APP_NODE_ROLE_DIAGNOSTIC=1
    )
elseif(APP_NODE_ROLE_LOWER STREQUAL "i2cdiag")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        APP_NODE_ROLE_I2CDIAG=1
    )
else()
    message(FATAL_ERROR
        "APP_NODE_ROLE must be diagnostic, sender, receiver, or i2cdiag "
        "(got '${APP_NODE_ROLE}')")
endif()
