#pragma once

#include "driver/uart.h"
#include <stdint.h>
#include <stdbool.h>

#pragma once

#include "esp_err.h"

typedef struct {
    uint16_t distance_mm;
    uint8_t signal_strength;
} ld2412_target_t;

typedef struct {
    uint8_t target_count;
    ld2412_target_t targets[8];  // Adjust size based on expected max targets
    uint8_t buf[64];             // Raw data buffer for debugging
    int start;
    int endData;                 // Index of last meaningful byte (e.g., calibration byte after 0x55)
} ld2412_frame_t;

typedef struct {
    uint8_t status_raw;
    uint16_t movement_distance_cm;
    uint8_t movement_energy;
    uint16_t static_distance_cm;
    uint8_t static_energy;
} ld2412_basic_target_t;

typedef struct {
    uint8_t target_id;
    uint16_t distance_cm;
    uint8_t signal_strength;
    uint8_t status_raw;
    bool motion;
    bool static_presence;
    bool is_new;
    bool is_lost;
    float motion_energy;
    float static_energy;
} ld2412_engineering_target_t;


#define LD2412_UART_PORT UART_NUM_1
#define LD2412_TX_PIN 16
#define LD2412_RX_PIN 17
#define LD2412_BAUD_RATE 115200


esp_err_t ld2412_init(void);
esp_err_t ld2412_enable_config(void);
esp_err_t ld2412_end_config(void);
esp_err_t ld2412_read_firmware_version(char *version_str, size_t max_len);
esp_err_t ld2412_start_stream(void);
esp_err_t ld2412_read_frame(ld2412_frame_t *frame);
esp_err_t ld2412_parse_basic(const uint8_t *buf, int start, int end);
esp_err_t ld2412_parse_engineering(const uint8_t *buf, int start, int end);
esp_err_t ld2412_parse_frame(ld2412_frame_t *frame);
const char* decode_status(uint8_t status );
esp_err_t ld2412_set_output_mode(uart_port_t uart_num, bool basic_mode);
esp_err_t ld2412_enable_configuration(uart_port_t uart_num);
esp_err_t ld2412_exit_configuration(uart_port_t uart_num);


