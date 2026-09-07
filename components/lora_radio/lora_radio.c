#include "lora_radio.h"

#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ra01s.h"

#define LORA_MAX_PAYLOAD_SIZE 255U
#define LORA_SPREADING_FACTOR 7U
#define LORA_BANDWIDTH_125_KHZ 4U
#define LORA_CODING_RATE_4_5 1U
#define LORA_PREAMBLE_LENGTH 8U

static void apply_lora_config(void)
{
    LoRaConfig(LORA_SPREADING_FACTOR,
               LORA_BANDWIDTH_125_KHZ,
               LORA_CODING_RATE_4_5,
               LORA_PREAMBLE_LENGTH,
               0,
               true,
               false);
}

esp_err_t lora_radio_init(void)
{
    LoRaInit();
    const int16_t result = LoRaBegin(LORA_RADIO_FREQUENCY_HZ,
                                     LORA_RADIO_TX_POWER_DBM,
                                     1.8f,
                                     true);
    if (result != ERR_NONE) {
        return ESP_FAIL;
    }

    apply_lora_config();
    return ESP_OK;
}

esp_err_t lora_radio_sleep(void)
{
    /* The driver exposes the command writer but no SetSleep() wrapper. */
    uint8_t config = SX126X_SLEEP_START_WARM | SX126X_SLEEP_RTC_OFF;
    WriteCommand(SX126X_CMD_SET_SLEEP, &config, 1);
    return ESP_OK;
}

esp_err_t lora_radio_wakeup(void)
{
    /* A sleeping SX1262 holds BUSY high, and every WriteCommand() waits for
     * BUSY to fall before it sends anything, so commands cannot be used to wake
     * the chip. Only a falling edge on NSS does that. spi_write_byte() is a raw
     * transfer with no BUSY wait, and the SPI driver owns NSS, so one dummy
     * byte produces the edge. The byte itself is discarded by the sleeping
     * chip. */
    uint8_t wake_byte = 0x00;
    spi_write_byte(&wake_byte, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    /* BUSY is low again, so ordinary commands work from here. */
    SetStandby(SX126X_STANDBY_RC);
    /* Warm start should retain these, but re-applying them makes the
     * post-sleep state deterministic and returns the radio to RX. */
    apply_lora_config();
    return ESP_OK;
}

esp_err_t lora_radio_send(const uint8_t *data, size_t length)
{
    if (!data || length == 0 || length > LORA_MAX_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t packet[LORA_MAX_PAYLOAD_SIZE];
    memcpy(packet, data, length);
    return LoRaSend(packet, (int16_t)length, SX126x_TXMODE_SYNC)
               ? ESP_OK
               : ESP_FAIL;
}

esp_err_t lora_radio_receive(uint8_t *data, size_t capacity,
                             size_t *received_length,
                             lora_rx_metadata_t *metadata)
{
    if (!data || !received_length || capacity == 0 ||
        capacity > LORA_MAX_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t length = LoRaReceive(data, (int16_t)capacity);
    if (length == 0) {
        *received_length = 0;
        return ESP_ERR_NOT_FOUND;
    }

    *received_length = length;
    if (metadata) {
        GetPacketStatus(&metadata->rssi_dbm, &metadata->snr_db);
    }
    return ESP_OK;
}
