#include "LD2412.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#define TAG "LD2412"
#define FRAME_HEADER_LEN 4
#define FRAME_TAIL_LEN 4
#define FRAME_MAX_LEN 64
#define UART_TIMEOUT_MS 100
static const uint8_t FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t FRAME_TAIL[]   = {0xF8, 0xF7, 0xF6, 0xF5};

static esp_err_t send_command(uint16_t cmd_word, const uint8_t *cmd_value, size_t value_len) {
    uint8_t frame[64];
    size_t idx = 0;

    memcpy(&frame[idx], FRAME_HEADER, 4); idx += 4;
    uint16_t data_len = 2 + value_len;
    frame[idx++] = data_len & 0xFF;
    frame[idx++] = (data_len >> 8) & 0xFF;
    frame[idx++] = cmd_word & 0xFF;
    frame[idx++] = (cmd_word >> 8) & 0xFF;
    if (cmd_value && value_len > 0) {
        memcpy(&frame[idx], cmd_value, value_len);
        idx += value_len;
    }
    memcpy(&frame[idx], FRAME_TAIL, 4); idx += 4;

    return uart_write_bytes(LD2412_UART_PORT, (const char *)frame, idx);
}

esp_err_t ld2412_init(void) {
    uart_config_t uart_config = {
        .baud_rate = LD2412_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(LD2412_UART_PORT, &uart_config);
    uart_set_pin(LD2412_UART_PORT, LD2412_TX_PIN, LD2412_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LD2412_UART_PORT, 1024, 0, 0, NULL, 0);
    return ESP_OK;
}

esp_err_t ld2412_enable_config(void) {
    uint8_t value[] = {0x01, 0x00};
    return send_command(0x00FF, value, sizeof(value));
}

esp_err_t ld2412_end_config(void) {
    return send_command(0x00FE, NULL, 0);
}

esp_err_t ld2412_read_firmware_version(char *version_str, size_t max_len) {
    send_command(0x00A0, NULL, 0);
    uint8_t response[64];
    int len = uart_read_bytes(LD2412_UART_PORT, response, sizeof(response), 100 / portTICK_PERIOD_MS);
    if (len < 20) return ESP_FAIL;

    // Parse firmware version from response
    uint16_t fw_type = response[8] | (response[9] << 8);
    uint16_t major   = response[10] | (response[11] << 8);
    uint32_t minor   = response[12] | (response[13] << 8) | (response[14] << 16) | (response[15] << 24);
    snprintf(version_str, max_len, "V%d.%d.%08lu", major, fw_type, (unsigned long)minor);

    return ESP_OK;
}

esp_err_t ld2412_start_stream(void) {
    const uint8_t start_cmd[] = {0xAA, 0x00, 0x01, 0x01};  // Example command
    return uart_write_bytes(UART_NUM_1, (const char *)start_cmd, sizeof(start_cmd));
}

esp_err_t ld2412_parse_frame(ld2412_frame_t *frame) {
    //ESP_LOGI(TAG, "frame start %d", frame->start);
    const uint8_t *buf = frame->buf;
    int start = frame->start;
    int end = frame->endData;

    // Validate payload closure
    if (buf[end - 1] != 0x55) {
        ESP_LOGE("LD2412", "Missing 0x55 marker before tail: found 0x%02X", buf[end - 1]);
        return ESP_FAIL;
    }
    uint8_t frame_type = buf[start];
    ESP_LOGI(TAG, "Frame type: 0x%02X", frame_type);
    if (frame_type == 0x02) {
        return ld2412_parse_basic(buf, start, end);
    } else if (frame_type == 0x01) {
        return ld2412_parse_engineering(buf, start, end);
    } else {
        ESP_LOGE("LD2412", "Unknown frame type: 0x%02X", frame_type);
        return ESP_FAIL;
    }
}

const char* decode_status(uint8_t status ) {
   
    static char status_str[64];
    int offset = snprintf(status_str, sizeof(status_str), "Target status: ");
    switch (status) {
    case 0x00:
        offset += snprintf(status_str + offset, sizeof(status_str)- offset, "untargeted\n");
        break;
    case 0x01:
        offset += snprintf(status_str + offset, sizeof(status_str)- offset, "moving target\n");
        break;
    case 0x02:
        offset += snprintf(status_str + offset, sizeof(status_str)- offset, "stationary target\n");
        break;
    case 0x03:
        offset += snprintf(status_str + offset, sizeof(status_str)- offset, "moving + stationary target\n");
        break;
    default:
        offset += snprintf(status_str + offset, sizeof(status_str)- offset, "unknown status\n");
        break;  
    }
    //ESP_LOGI("Debug", "Offset %d  status %d  %s", offset, status, status_str);
    return status_str;
}

esp_err_t ld2412_parse_basic(const uint8_t *buf, int start, int end) {
    ld2412_basic_target_t target;
    const char* target_status = decode_status(buf[start+2]);

    target.movement_distance_cm = buf[start + 3] | (buf[start + 4] << 8);
    target.movement_energy = buf[start + 5];

    target.static_distance_cm = buf[start + 6] | (buf[start + 7] << 8);
    target.static_energy = buf[start + 8];

    ESP_LOGI("LD2412", "mark char 0x%x",buf[start+1]);

    ESP_LOGI("LD2412", "TS: %d -> %s", buf[start+2], target_status);

    ESP_LOGI("LD2412", "Movement: %d cm | Energy: %d", target.movement_distance_cm, target.movement_energy);
    ESP_LOGI("LD2412", "Static:   %d cm | Energy: %d", target.static_distance_cm, target.static_energy);

    return ESP_OK;
}

esp_err_t ld2412_parse_engineering(const uint8_t *buf, int start, int end) {
    ESP_LOGI("LD2412", "Engineering frame detected — parsing not yet implemented");
    return ESP_OK;
}

esp_err_t ld2412_read_frame(ld2412_frame_t *frame) {
    uint8_t temp_buf[FRAME_MAX_LEN];
    int len = uart_read_bytes(UART_NUM_1, temp_buf, sizeof(temp_buf), UART_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (len < 10) {
        ESP_LOGE(TAG, "UART read too short (%d bytes)", len);
        return ESP_FAIL;
    }

    for (int i = 0; i <= len - 10; i++) {
        // Look for header
        if (memcmp(&temp_buf[i], FRAME_HEADER, FRAME_HEADER_LEN) != 0) continue;

        // Check if we have enough for length field
        if (i + FRAME_HEADER_LEN + 2 > len) {
            ESP_LOGE(TAG, "Frame too short to read length");
            return ESP_FAIL;
        }

        uint16_t payload_len = temp_buf[i + 4] | (temp_buf[i + 5] << 8);
        int full_frame_len = FRAME_HEADER_LEN + 2 + payload_len + FRAME_TAIL_LEN;

        if (i + full_frame_len > len) {
            ESP_LOGE(TAG, "Incomplete frame: expected %d bytes, got %d", full_frame_len, len - i);
            return ESP_FAIL;
        }

        // Check tail
        if (memcmp(&temp_buf[i + full_frame_len - FRAME_TAIL_LEN], FRAME_TAIL, FRAME_TAIL_LEN) != 0) {
            ESP_LOGE(TAG, "Frame tail mismatch at index %d", i + full_frame_len - FRAME_TAIL_LEN);
            return ESP_FAIL;
        }

        // Copy valid frame
        memset(frame->buf, 0, sizeof(frame->buf));  // Clear buffer
        memcpy(frame->buf, &temp_buf[i], full_frame_len);
        frame->start = 6;
        frame->target_count = 0;
        frame->endData = frame->start+payload_len-1;  // last byte of the frame of payload
        // Optional: zero out any trailing bytes beyond the frame
        for (int j = full_frame_len; j < sizeof(frame->buf); j++) {
            frame->buf[j] = 0;
        }

        return ESP_OK;
    }

    ESP_LOGE(TAG, "No valid frame found in %d bytes", len);
    return ESP_FAIL;
}

esp_err_t ld2412_set_output_mode(uart_port_t uart_num, bool basic_mode) {
    uint8_t enMode = basic_mode ? 0x63 : 0x62;

    uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,       // Header
        0x02, 0x00,                   // Payload length = 5
        enMode, 0x00,                  // 
        0x04, 0x03, 0x02, 0x01        // Tail
    };

    ESP_LOGI("LD2412", "Sending command to set mode: %s", basic_mode ? "Basic" : "Engineering");
    uart_write_bytes(uart_num, (const char *)cmd, sizeof(cmd));

    // Wait for acknowledgment frame (optional: adjust timeout)
    uint8_t ack_buf[1023];
    int len = uart_read_bytes(uart_num, ack_buf, sizeof(ack_buf), 100 / portTICK_PERIOD_MS);

    uint8_t ack[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x04, 0x00,
        enMode, 0x01,
        0x00, 0x00,
        0x04, 0x03, 0x02, 0x01
    };
    
     for (int i = 0; i <= len - sizeof(ack); i++) {
         if (memcmp(&ack_buf[i], ack, sizeof(ack)) == 0){
             ESP_LOGI("LD2412", "ACK: report mode %d", i);
             return ESP_OK;
         }}

    ESP_LOGW("LD2412", "No valid ACK for setting report mode");

    return ESP_FAIL;
    }

esp_err_t ld2412_enable_configuration(uart_port_t uart_num) {
    uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x04, 0x00,
        0xFF, 0x00,
        0x01, 0x00,
        0x04, 0x03, 0x02, 0x01
    };
    
    ESP_LOGI("LD2412", "Sending ENABLE configuration command");
    uart_write_bytes(uart_num, (const char *)cmd, sizeof(cmd));
    ESP_LOGI("LD2412", "Send DONE");
    uint8_t ack_buf[1023];
    for(int i=0; i<sizeof(ack_buf); i++){
        ack_buf[i]=0;
    }
    int len = uart_read_bytes(uart_num, ack_buf, sizeof(ack_buf), 200 / portTICK_PERIOD_MS);
    // ESP_LOG_BUFFER_HEX(TAG, ack_buf, sizeof(ack_buf));

uint8_t ack[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x08, 0x00,
        0xFF, 0x01,
        0x00, 0x00,
        0x01, 0x00,
        0x00, 0x00,
        0x04, 0x03, 0x02, 0x01
    };
     for (int i = 0; i <= len - sizeof(ack); i++) {
         if (memcmp(&ack_buf[i], ack, sizeof(ack)) == 0){
             ESP_LOGI("LD2412", "ACK: Configuration ENABLED %d", i);
             return ESP_OK;
         }}

    ESP_LOGW("LD2412", "No valid ACK for ENABLE configuration");
    return ESP_FAIL;
}

esp_err_t ld2412_exit_configuration(uart_port_t uart_num) {
    uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xFE, 0x00,
        0x04, 0x03, 0x02, 0x01
    };

    ESP_LOGI("LD2412", "Sending EXIT configuration command");
    uart_write_bytes(uart_num, (const char *)cmd, sizeof(cmd));

    uint8_t ack_buf[1023];
    int len = uart_read_bytes(uart_num, ack_buf, sizeof(ack_buf), 200 / portTICK_PERIOD_MS);

    uint8_t ack[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x04, 0x00,
        0xFE, 0x01,
        0x00, 0x00,
        0x04, 0x03, 0x02, 0x01
    };

     for (int i = 0; i <= len - sizeof(ack); i++) {
         if (memcmp(&ack_buf[i], ack, sizeof(ack)) == 0){
             ESP_LOGI("LD2412", "ACK: EXIT Configuration mode %d", i);
             return ESP_OK;
         }
        }

    ESP_LOGW("LD2412", "No valid ACK for EXIT configuration");
    return ESP_FAIL;
}
