#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ST7789.h"
#include "SD_SPI.h"
#include "RGB.h"
#include "LVGL_Example.h"
#include "LD2412/LD2412.h"

#include "lvgl.h"

#define TAG "MAIN"

#define LVGL_PERIOD_MS   10
#define UI_PERIOD_MS     200
#define NO_TARGET_MS     2000   /* declare the sensor silent after this long without a frame */
#define LABEL_WIDTH      (EXAMPLE_LCD_H_RES - 8)

static void init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static lv_obj_t *create_label(const char *text, lv_coord_t y_offset) {
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_obj_set_width(label, LABEL_WIDTH);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, text);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, y_offset);
    return label;
}

static void log_firmware_version(void) {
    char fw[32];

    if (ld2412_enable_config() != ESP_OK) {
        ESP_LOGE(TAG, "LD2412 did not enter configuration mode");
        return;
    }
    if (ld2412_read_firmware_version(fw, sizeof(fw)) == ESP_OK) {
        ESP_LOGI(TAG, "LD2412 firmware: %s", fw);
    } else {
        ESP_LOGE(TAG, "Failed to read LD2412 firmware version");
    }
    /* Engineering mode is off after power-on; pass true to also get per-gate energies. */
    ESP_ERROR_CHECK_WITHOUT_ABORT(ld2412_set_engineering_mode(false));
    ESP_ERROR_CHECK_WITHOUT_ABORT(ld2412_end_config());
}

void app_main(void) {
    init_nvs();

    ESP_ERROR_CHECK(ld2412_init());
    log_firmware_version();

    Flash_Searching();
    RGB_Init();
    RGB_Example();
    SD_Init();
    LCD_Init();
    BK_Light(50);
    LVGL_Init();

    lv_obj_t *state_label  = create_label("Sensor: waiting", -40);
    lv_obj_t *dist_label   = create_label("Distance: --.- m", 0);
    lv_obj_t *energy_label = create_label("Energy: ---", 40);

    ld2412_data_t data = {0};
    TickType_t last_frame = 0;
    TickType_t last_ui = 0;

    while (1) {
        /* Drain whatever the radar sent since the last pass; it reports at ~10 Hz. */
        while (ld2412_read_data(&data, 0) == ESP_OK) {
            last_frame = xTaskGetTickCount();
        }

        const TickType_t now = xTaskGetTickCount();
        if (now - last_ui >= pdMS_TO_TICKS(UI_PERIOD_MS)) {
            last_ui = now;
            char buf[48];

            if (last_frame == 0 || now - last_frame > pdMS_TO_TICKS(NO_TARGET_MS)) {
                lv_label_set_text(state_label, "Sensor: no data");
                lv_label_set_text(dist_label, "Distance: --.- m");
                lv_label_set_text(energy_label, "Energy: ---");
            } else {
                snprintf(buf, sizeof(buf), "%s", ld2412_state_str(data.state));
                lv_label_set_text(state_label, buf);

                const bool moving = (data.state == LD2412_TARGET_MOVING ||
                                     data.state == LD2412_TARGET_BOTH);
                const uint16_t distance_cm = moving ? data.moving_distance_cm
                                                    : data.static_distance_cm;
                const uint8_t energy = moving ? data.moving_energy : data.static_energy;

                if (data.state == LD2412_TARGET_NONE) {
                    lv_label_set_text(dist_label, "Distance: --.- m");
                    lv_label_set_text(energy_label, "Energy: ---");
                } else {
                    snprintf(buf, sizeof(buf), "Distance: %4.2f m", distance_cm / 100.0f);
                    lv_label_set_text(dist_label, buf);
                    snprintf(buf, sizeof(buf), "Energy: %3u (mov %3u / sta %3u)",
                             energy, data.moving_energy, data.static_energy);
                    lv_label_set_text(energy_label, buf);
                }
            }
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(LVGL_PERIOD_MS));
    }
}
