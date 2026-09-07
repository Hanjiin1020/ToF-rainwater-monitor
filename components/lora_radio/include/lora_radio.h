#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_RADIO_FREQUENCY_HZ 922100000UL

/* 5 dBm, matching the Arduino sketch that ran these boards from battery alone.
 * 10 dBm draws noticeably more during transmit, and the very first packet goes
 * out right after boot while the supply is still settling - enough to brown out
 * a partly charged cell before the packet completes, which looks like total
 * silence at A. Measured RSSI here is -14..-27 dBm against a sensitivity floor
 * near -120 dBm, so giving up 5 dB costs nothing and eases receiver saturation
 * when the boards sit close together. */
#define LORA_RADIO_TX_POWER_DBM 5

typedef struct {
    int8_t rssi_dbm;
    int8_t snr_db;
} lora_rx_metadata_t;

esp_err_t lora_radio_init(void);
esp_err_t lora_radio_send(const uint8_t *data, size_t length);
esp_err_t lora_radio_receive(uint8_t *data, size_t capacity,
                             size_t *received_length,
                             lora_rx_metadata_t *metadata);

/* SX1262 warm-start sleep: the modulation and packet configuration survive, so
 * lora_radio_wakeup() costs far less than a full init. Nothing is received
 * while asleep, so only sleep outside the wake window. */
esp_err_t lora_radio_sleep(void);

/* Brings the radio back to standby and re-applies the LoRa configuration. If it
 * returns anything but ESP_OK the caller should fall back to lora_radio_init(). */
esp_err_t lora_radio_wakeup(void);

#ifdef __cplusplus
}
#endif
