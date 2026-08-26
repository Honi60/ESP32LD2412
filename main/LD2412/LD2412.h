#pragma once

#include "driver/uart.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Number of distance gates reported in engineering mode (gate 0 .. 13). */
#define LD2412_GATE_COUNT 14

#ifndef CONFIG_LD2412_UART_PORT
#define CONFIG_LD2412_UART_PORT 1
#endif
#ifndef CONFIG_LD2412_TX_GPIO
#define CONFIG_LD2412_TX_GPIO 16
#endif
#ifndef CONFIG_LD2412_RX_GPIO
#define CONFIG_LD2412_RX_GPIO 17
#endif

#define LD2412_UART_PORT ((uart_port_t)CONFIG_LD2412_UART_PORT)
#define LD2412_TX_PIN    CONFIG_LD2412_TX_GPIO  /* ESP32 TX -> radar RX */
#define LD2412_RX_PIN    CONFIG_LD2412_RX_GPIO  /* ESP32 RX <- radar TX */
#define LD2412_BAUD_RATE 115200

/* Target state values, protocol table 12. */
typedef enum {
    LD2412_TARGET_NONE   = 0x00,
    LD2412_TARGET_MOVING = 0x01,
    LD2412_TARGET_STATIC = 0x02,
    LD2412_TARGET_BOTH   = 0x03,
} ld2412_target_state_t;

/* One decoded report frame, protocol tables 11 and 13. */
typedef struct {
    ld2412_target_state_t state;
    uint16_t moving_distance_cm;
    uint8_t  moving_energy;
    uint16_t static_distance_cm;
    uint8_t  static_energy;
    bool     engineering;                              /* fields below are valid */
    uint8_t  max_moving_gate;
    uint8_t  max_static_gate;
    uint8_t  moving_gate_energy[LD2412_GATE_COUNT];
    uint8_t  static_gate_energy[LD2412_GATE_COUNT];
} ld2412_data_t;

esp_err_t ld2412_init(void);

/* Configuration commands. Every command must be wrapped in
 * ld2412_enable_config() / ld2412_end_config(), protocol section 2.4.1. */
esp_err_t ld2412_enable_config(void);
esp_err_t ld2412_end_config(void);
esp_err_t ld2412_set_engineering_mode(bool enable);
esp_err_t ld2412_read_firmware_version(char *version_str, size_t max_len);

/* Reads the next report frame, waiting at most timeout_ms.
 * Returns ESP_ERR_TIMEOUT when no complete frame arrived. */
esp_err_t ld2412_read_data(ld2412_data_t *out, uint32_t timeout_ms);

const char *ld2412_state_str(ld2412_target_state_t state);
