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
/* GPIO16/17 are U0TXD/U0RXD and belong to the console; 18/19 are free on the
 * ESP32-C6-LCD-1.47 header. */
#ifndef CONFIG_LD2412_TX_GPIO
#define CONFIG_LD2412_TX_GPIO 18
#endif
#ifndef CONFIG_LD2412_RX_GPIO
#define CONFIG_LD2412_RX_GPIO 19
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

/* Distance gate resolution, protocol section 2.2.3. Gates are 0.75 m wide
 * by default; changing the resolution needs a radar reboot to take effect. */
typedef enum {
    LD2412_RESOLUTION_75CM = 0,
    LD2412_RESOLUTION_50CM = 1,
    LD2412_RESOLUTION_20CM = 3,
} ld2412_resolution_t;

/* Basic parameters, protocol sections 2.2.5 and 2.2.6. Distances are in gates;
 * no_one_duration_s is how long the radar keeps reporting a target after it
 * is lost, in seconds. */
typedef struct {
    uint8_t  min_gate;
    uint8_t  max_gate;
    uint16_t no_one_duration_s;
    uint8_t  out_polarity;   /* 0: occupied drives OUT high; 1: unoccupied drives it low */
} ld2412_basic_params_t;

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

/* Logs the first bytes the radar sends on its own; ESP_ERR_TIMEOUT if the line stays silent. */
esp_err_t ld2412_probe(uint32_t timeout_ms);

/* Configuration commands. Every command must be wrapped in
 * ld2412_enable_config() / ld2412_end_config(), protocol section 2.4.1. */
esp_err_t ld2412_enable_config(void);
esp_err_t ld2412_end_config(void);
esp_err_t ld2412_set_engineering_mode(bool enable);
esp_err_t ld2412_read_firmware_version(char *version_str, size_t max_len);

/* Gate configuration, protocol sections 2.2.3-2.2.6 and 2.2.9-2.2.12.
 * Gate indexes run 0..LD2412_GATE_COUNT-1 and each gate is ld2412_resolution_t
 * wide (75 cm by default). A sensitivity of 100 masks a gate off completely.
 * Sensitivity and basic parameters persist on the radar across reboots;
 * the resolution applies only after one. All of these must be called between
 * ld2412_enable_config() and ld2412_end_config(). */
esp_err_t ld2412_set_resolution(ld2412_resolution_t resolution);
esp_err_t ld2412_get_resolution(ld2412_resolution_t *resolution);
esp_err_t ld2412_set_basic_params(const ld2412_basic_params_t *params);
esp_err_t ld2412_get_basic_params(ld2412_basic_params_t *params);
esp_err_t ld2412_set_motion_sensitivity(const uint8_t per_gate[LD2412_GATE_COUNT]);
esp_err_t ld2412_get_motion_sensitivity(uint8_t per_gate[LD2412_GATE_COUNT]);
esp_err_t ld2412_set_static_sensitivity(const uint8_t per_gate[LD2412_GATE_COUNT]);
esp_err_t ld2412_get_static_sensitivity(uint8_t per_gate[LD2412_GATE_COUNT]);

/* Reads the next report frame, waiting at most timeout_ms.
 * Returns ESP_ERR_TIMEOUT when no complete frame arrived. */
esp_err_t ld2412_read_data(ld2412_data_t *out, uint32_t timeout_ms);

const char *ld2412_state_str(ld2412_target_state_t state);
