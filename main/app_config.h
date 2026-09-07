#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "tof_driver.h"

/* Verify these placeholders against the actual harness before flashing. */
#define TOF_I2C_PORT I2C_NUM_0
#define TOF_PIN_SDA GPIO_NUM_41
#define TOF_PIN_SCL GPIO_NUM_42
#define TOF_PIN_RESET GPIO_NUM_NC
#define TOF_I2C_SPEED_HZ 1000000U

#define TOF_DEFAULT_ADDRESS_7BIT 0x29U
#define HEIGHT_DEMO_PLACEHOLDER_BASELINE_MM 1000U

/* Heltec WiFi LoRa 32 V3: Vext is enabled by driving GPIO36 low. */
#define BOARD_VEXT_CONTROL_GPIO GPIO_NUM_36
#define BOARD_VEXT_ENABLED_LEVEL 0

#define LORA_RECEIVE_POLL_MS 10U
#define TOF_INIT_RETRY_COUNT 3U
#define TOF_INIT_RETRY_DELAY_MS 500U
#define TOF_FRAME_TIMEOUT_MS 3000U

/* Shared by every role so the pin and address settings live in one place. */
static inline tof_config_t app_default_tof_config(void)
{
    return (tof_config_t){
        .i2c_port = TOF_I2C_PORT,
        .sda_gpio = TOF_PIN_SDA,
        .scl_gpio = TOF_PIN_SCL,
        .reset_gpio = TOF_PIN_RESET,
        .address_7bit = TOF_DEFAULT_ADDRESS_7BIT,
        .i2c_speed_hz = TOF_I2C_SPEED_HZ,
        /* Lower is better here: a longer integration window gets more
         * photons back from dark surfaces, and we only keep one frame per wake
         * so the slower rate costs nothing. The Arduino sketch that worked on
         * this hardware also used 5 Hz. */
        .ranging_hz = 5,
    };
}
