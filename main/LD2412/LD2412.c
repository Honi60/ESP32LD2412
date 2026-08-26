#include "LD2412.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "LD2412"

/* Report frames, protocol table 8. */
static const uint8_t DATA_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t DATA_TAIL[4]   = {0xF8, 0xF7, 0xF6, 0xF5};
/* Command / ACK frames, protocol section 2.1. */
static const uint8_t CMD_HEADER[4]  = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_TAIL[4]    = {0x04, 0x03, 0x02, 0x01};

#define HEADER_LEN        4
#define TAIL_LEN          4
#define LEN_FIELD_LEN     2
#define FRAME_OVERHEAD    (HEADER_LEN + LEN_FIELD_LEN + TAIL_LEN)
#define MAX_PAYLOAD_LEN   128
#define UART_RX_BUF_SIZE  1024

#define CMD_ENABLE_CONFIG      0x00FF
#define CMD_END_CONFIG         0x00FE
#define CMD_ENGINEERING_ON     0x0062
#define CMD_ENGINEERING_OFF    0x0063
#define CMD_READ_FW_VERSION    0x00A0

#define DATA_TYPE_ENGINEERING  0x01
#define DATA_TYPE_BASIC        0x02
#define DATA_INTRA_HEAD        0xAA
#define DATA_INTRA_END         0x55

#define BASIC_PAYLOAD_LEN        11  /* type + AA + 9 data + 55 + check */
#define ENGINEERING_PAYLOAD_LEN  (BASIC_PAYLOAD_LEN + 2 + 2 * LD2412_GATE_COUNT)

/* Bytes received but not yet consumed as a complete frame. */
static uint8_t s_rx[2 * (FRAME_OVERHEAD + MAX_PAYLOAD_LEN)];
static size_t s_rx_len;

static uint16_t le16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

esp_err_t ld2412_init(void) {
    const uart_config_t uart_config = {
        .baud_rate  = LD2412_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(LD2412_UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(LD2412_UART_PORT, &uart_config),
                        TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(LD2412_UART_PORT, LD2412_TX_PIN, LD2412_RX_PIN,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");
    s_rx_len = 0;
    return ESP_OK;
}

/* Builds and sends a command frame: FD FC FB FA | len | cmd | value | 04 03 02 01. */
static esp_err_t send_command(uint16_t cmd_word, const uint8_t *value, size_t value_len) {
    uint8_t frame[FRAME_OVERHEAD + 2 + MAX_PAYLOAD_LEN];
    size_t idx = 0;

    if (value_len > MAX_PAYLOAD_LEN) return ESP_ERR_INVALID_ARG;

    memcpy(&frame[idx], CMD_HEADER, HEADER_LEN);
    idx += HEADER_LEN;

    const uint16_t data_len = (uint16_t)(2 + value_len);
    frame[idx++] = data_len & 0xFF;
    frame[idx++] = (data_len >> 8) & 0xFF;
    frame[idx++] = cmd_word & 0xFF;
    frame[idx++] = (cmd_word >> 8) & 0xFF;
    if (value != NULL && value_len > 0) {
        memcpy(&frame[idx], value, value_len);
        idx += value_len;
    }
    memcpy(&frame[idx], CMD_TAIL, TAIL_LEN);
    idx += TAIL_LEN;

    const int written = uart_write_bytes(LD2412_UART_PORT, (const char *)frame, idx);
    return (written == (int)idx) ? ESP_OK : ESP_FAIL;
}

/* Waits for the ACK of cmd_word. Report frames arriving in between are skipped.
 * On success, value/value_len receive the ACK return value (status word included). */
static esp_err_t wait_ack(uint16_t cmd_word, uint8_t *value, size_t *value_len, uint32_t timeout_ms) {
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    uint8_t buf[FRAME_OVERHEAD + MAX_PAYLOAD_LEN];
    size_t len = 0;

    while (xTaskGetTickCount() < deadline) {
        const int read = uart_read_bytes(LD2412_UART_PORT, buf + len, sizeof(buf) - len,
                                         pdMS_TO_TICKS(20));
        if (read > 0) len += (size_t)read;

        for (size_t i = 0; i + FRAME_OVERHEAD <= len; i++) {
            if (memcmp(&buf[i], CMD_HEADER, HEADER_LEN) != 0) continue;

            const uint16_t payload_len = le16(&buf[i + HEADER_LEN]);
            const size_t frame_len = FRAME_OVERHEAD + payload_len;
            if (payload_len < 4 || payload_len > MAX_PAYLOAD_LEN) continue;
            if (i + frame_len > len) break;  /* wait for the rest */
            if (memcmp(&buf[i + frame_len - TAIL_LEN], CMD_TAIL, TAIL_LEN) != 0) continue;

            const uint8_t *payload = &buf[i + HEADER_LEN + LEN_FIELD_LEN];
            if (le16(payload) != (uint16_t)(cmd_word | 0x0100)) continue;
            if (le16(payload + 2) != 0) {
                ESP_LOGE(TAG, "command 0x%04X failed, status %u", cmd_word, le16(payload + 2));
                return ESP_FAIL;
            }
            if (value != NULL && value_len != NULL) {
                const size_t copy = (payload_len - 2 < *value_len) ? payload_len - 2 : *value_len;
                memcpy(value, payload + 2, copy);
                *value_len = copy;
            }
            return ESP_OK;
        }

        /* Drop everything before the last possible header start so the buffer cannot fill up. */
        if (len > sizeof(buf) - HEADER_LEN) {
            memmove(buf, buf + len - HEADER_LEN, HEADER_LEN);
            len = HEADER_LEN;
        }
    }

    ESP_LOGW(TAG, "no ACK for command 0x%04X", cmd_word);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t send_command_wait_ack(uint16_t cmd_word, const uint8_t *value, size_t value_len,
                                       uint8_t *ack_value, size_t *ack_value_len) {
    ESP_RETURN_ON_ERROR(send_command(cmd_word, value, value_len), TAG, "send 0x%04X failed", cmd_word);
    return wait_ack(cmd_word, ack_value, ack_value_len, 500);
}

esp_err_t ld2412_enable_config(void) {
    const uint8_t value[] = {0x01, 0x00};
    /* Stale report frames confuse ACK matching far less if the queue is empty. */
    uart_flush_input(LD2412_UART_PORT);
    s_rx_len = 0;
    return send_command_wait_ack(CMD_ENABLE_CONFIG, value, sizeof(value), NULL, NULL);
}

esp_err_t ld2412_end_config(void) {
    return send_command_wait_ack(CMD_END_CONFIG, NULL, 0, NULL, NULL);
}

esp_err_t ld2412_set_engineering_mode(bool enable) {
    const uint16_t cmd = enable ? CMD_ENGINEERING_ON : CMD_ENGINEERING_OFF;
    return send_command_wait_ack(cmd, NULL, 0, NULL, NULL);
}

esp_err_t ld2412_read_firmware_version(char *version_str, size_t max_len) {
    /* ACK value: 2 status + 2 firmware type + 2 major + 4 minor. */
    uint8_t value[16];
    size_t value_len = sizeof(value);

    ESP_RETURN_ON_ERROR(send_command_wait_ack(CMD_READ_FW_VERSION, NULL, 0, value, &value_len),
                        TAG, "read firmware version failed");
    if (value_len < 8) return ESP_ERR_INVALID_RESPONSE;

    /* value[0..1] is the firmware type (0x2412), the version digits are BCD-like:
     * 10 01 10 18 04 24 reads as V1.10.24041810. */
    snprintf(version_str, max_len, "V%X.%02X.%02X%02X%02X%02X",
             value[3], value[2], value[7], value[6], value[5], value[4]);
    return ESP_OK;
}

const char *ld2412_state_str(ld2412_target_state_t state) {
    switch (state) {
    case LD2412_TARGET_NONE:   return "no target";
    case LD2412_TARGET_MOVING: return "moving";
    case LD2412_TARGET_STATIC: return "static";
    case LD2412_TARGET_BOTH:   return "moving+static";
    default:                   return "unknown";
    }
}

/* Decodes one intra-frame payload (data type .. check byte) into out. */
static esp_err_t parse_payload(const uint8_t *payload, size_t payload_len, ld2412_data_t *out) {
    if (payload_len < BASIC_PAYLOAD_LEN) {
        ESP_LOGE(TAG, "payload too short (%u bytes)", (unsigned)payload_len);
        return ESP_ERR_INVALID_SIZE;
    }
    if (payload[1] != DATA_INTRA_HEAD || payload[payload_len - 2] != DATA_INTRA_END) {
        ESP_LOGE(TAG, "bad intra-frame markers 0x%02X/0x%02X", payload[1], payload[payload_len - 2]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(out, 0, sizeof(*out));
    out->state               = (ld2412_target_state_t)payload[2];
    out->moving_distance_cm  = le16(&payload[3]);
    out->moving_energy       = payload[5];
    out->static_distance_cm  = le16(&payload[6]);
    out->static_energy       = payload[8];

    if (payload[0] == DATA_TYPE_BASIC) {
        return ESP_OK;
    }
    if (payload[0] != DATA_TYPE_ENGINEERING) {
        ESP_LOGE(TAG, "unknown data type 0x%02X", payload[0]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (payload_len < ENGINEERING_PAYLOAD_LEN) {
        ESP_LOGE(TAG, "engineering payload too short (%u bytes)", (unsigned)payload_len);
        return ESP_ERR_INVALID_SIZE;
    }

    out->engineering     = true;
    out->max_moving_gate = payload[9];
    out->max_static_gate = payload[10];
    memcpy(out->moving_gate_energy, &payload[11], LD2412_GATE_COUNT);
    memcpy(out->static_gate_energy, &payload[11 + LD2412_GATE_COUNT], LD2412_GATE_COUNT);
    return ESP_OK;
}

/* Consumes the first complete report frame in s_rx, if any. */
static bool take_frame(ld2412_data_t *out, esp_err_t *parse_result) {
    for (size_t i = 0; i + FRAME_OVERHEAD <= s_rx_len; i++) {
        if (memcmp(&s_rx[i], DATA_HEADER, HEADER_LEN) != 0) continue;

        const uint16_t payload_len = le16(&s_rx[i + HEADER_LEN]);
        if (payload_len > MAX_PAYLOAD_LEN) {
            /* Bogus length: skip this header and keep looking. */
            continue;
        }
        const size_t frame_len = FRAME_OVERHEAD + payload_len;
        if (i + frame_len > s_rx_len) {
            /* Incomplete: keep the partial frame for the next read. */
            memmove(s_rx, &s_rx[i], s_rx_len - i);
            s_rx_len -= i;
            return false;
        }
        if (memcmp(&s_rx[i + frame_len - TAIL_LEN], DATA_TAIL, TAIL_LEN) != 0) continue;

        *parse_result = parse_payload(&s_rx[i + HEADER_LEN + LEN_FIELD_LEN], payload_len, out);
        const size_t consumed = i + frame_len;
        memmove(s_rx, &s_rx[consumed], s_rx_len - consumed);
        s_rx_len -= consumed;
        return true;
    }

    /* No header found: keep only the last few bytes, a header may be split across reads. */
    if (s_rx_len > HEADER_LEN - 1) {
        memmove(s_rx, &s_rx[s_rx_len - (HEADER_LEN - 1)], HEADER_LEN - 1);
        s_rx_len = HEADER_LEN - 1;
    }
    return false;
}

esp_err_t ld2412_read_data(ld2412_data_t *out, uint32_t timeout_ms) {
    if (out == NULL) return ESP_ERR_INVALID_ARG;

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    const TickType_t read_ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(20);
    do {
        esp_err_t parse_result;
        if (take_frame(out, &parse_result)) return parse_result;

        const int read = uart_read_bytes(LD2412_UART_PORT, s_rx + s_rx_len,
                                         sizeof(s_rx) - s_rx_len, read_ticks);
        if (read > 0) s_rx_len += (size_t)read;
    } while (xTaskGetTickCount() < deadline);

    return ESP_ERR_TIMEOUT;
}
