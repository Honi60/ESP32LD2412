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
#define LOG_PERIOD_MS    1000   /* mirror the readings on the console */
#define NO_TARGET_MS     2000   /* declare the sensor silent after this long without a frame */

#define HALF_WIDTH       (EXAMPLE_LCD_H_RES / 2)
#define STATUS_Y         2      /* connection row */
#define SIGNAL_Y         30     /* per-side energy row */
#define VALUE_Y          76     /* the big distance digits */

#define COLOR_STATIC     lv_color_hex(0x00E000)
#define COLOR_MOVING     lv_color_hex(0xB040FF)
#define COLOR_OK         lv_color_hex(0x00E000)
#define COLOR_FAIL       lv_color_hex(0xFF2020)

LV_FONT_DECLARE(font_digits_100);
LV_FONT_DECLARE(font_text_20);

static void init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

/* One half-width, centred line of text; x picks the left or the right column. */
static lv_obj_t *create_label(lv_coord_t x, lv_coord_t y, const lv_font_t *font,
                              lv_color_t color, const char *text) {
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_obj_set_width(label, HALF_WIDTH);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, color, 0);
    lv_label_set_text(label, text);
    lv_obj_set_pos(label, x, y);
    return label;
}

/* Metres with one decimal, but 4 glyphs do not fit in a half screen, so past 10 m
 * (the last gate is at 10.5 m) the decimal is dropped. */
static void set_distance(lv_obj_t *label, bool valid, uint16_t distance_cm) {
    char buf[8];

    if (!valid) {
        lv_label_set_text(label, "-.-");
    } else if (distance_cm >= 1000) {
        snprintf(buf, sizeof(buf), "%u", distance_cm / 100);
        lv_label_set_text(label, buf);
    } else {
        snprintf(buf, sizeof(buf), "%.1f", distance_cm / 100.0f);
        lv_label_set_text(label, buf);
    }
}

static void create_divider(void) {
    static lv_point_t points[] = {{HALF_WIDTH, SIGNAL_Y}, {HALF_WIDTH, EXAMPLE_LCD_V_RES}};
    lv_obj_t *line = lv_line_create(lv_scr_act());
    lv_line_set_points(line, points, 2);
    lv_obj_set_style_line_width(line, 2, 0);
    lv_obj_set_style_line_color(line, lv_color_hex(0x808080), 0);
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
    /* Reports come unprompted, so a silent line here means wiring, not protocol. */
    if (ld2412_probe(1000) == ESP_OK) {
        log_firmware_version();
    }

    Flash_Searching();
    RGB_Init();
    RGB_Example();
    SD_Init();
    LCD_Init();
    BK_Light(50);
    LVGL_Init();

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

    lv_obj_t *conn_label = create_label(HALF_WIDTH / 2, STATUS_Y, &font_text_20,
                                        COLOR_FAIL, "connect X");
    lv_obj_t *static_sig = create_label(0, SIGNAL_Y, &font_text_20,
                                        COLOR_STATIC, "static sig ---");
    lv_obj_t *moving_sig = create_label(HALF_WIDTH, SIGNAL_Y, &font_text_20,
                                        COLOR_MOVING, "moving sig ---");
    lv_obj_t *static_val = create_label(0, VALUE_Y, &font_digits_100, COLOR_STATIC, "-.-");
    lv_obj_t *moving_val = create_label(HALF_WIDTH, VALUE_Y, &font_digits_100, COLOR_MOVING, "-.-");
    create_divider();

    ld2412_data_t data = {0};
    TickType_t last_frame = 0;
    TickType_t last_ui = 0;
    TickType_t last_log = 0;

    while (1) {
        /* Drain whatever the radar sent since the last pass; it reports at ~10 Hz. */
        while (ld2412_read_data(&data, 0) == ESP_OK) {
            last_frame = xTaskGetTickCount();
        }

        const TickType_t now = xTaskGetTickCount();
        const bool connected = (last_frame != 0 && now - last_frame <= pdMS_TO_TICKS(NO_TARGET_MS));

        if (now - last_ui >= pdMS_TO_TICKS(UI_PERIOD_MS)) {
            last_ui = now;
            char buf[24];

            lv_label_set_text(conn_label, connected ? "connect V" : "connect X");
            lv_obj_set_style_text_color(conn_label, connected ? COLOR_OK : COLOR_FAIL, 0);

            if (!connected) {
                lv_label_set_text(static_sig, "static sig ---");
                lv_label_set_text(moving_sig, "moving sig ---");
                set_distance(static_val, false, 0);
                set_distance(moving_val, false, 0);
            } else {
                const bool has_static = (data.state == LD2412_TARGET_STATIC ||
                                         data.state == LD2412_TARGET_BOTH);
                const bool has_moving = (data.state == LD2412_TARGET_MOVING ||
                                         data.state == LD2412_TARGET_BOTH);

                snprintf(buf, sizeof(buf), "static sig %3u", data.static_energy);
                lv_label_set_text(static_sig, buf);
                snprintf(buf, sizeof(buf), "moving sig %3u", data.moving_energy);
                lv_label_set_text(moving_sig, buf);

                set_distance(static_val, has_static, data.static_distance_cm);
                set_distance(moving_val, has_moving, data.moving_distance_cm);
            }
        }

        if (now - last_log >= pdMS_TO_TICKS(LOG_PERIOD_MS)) {
            last_log = now;
            if (!connected) {
                ESP_LOGW(TAG, "no radar frame for %d ms", NO_TARGET_MS);
            } else {
                ESP_LOGI(TAG, "%s: moving %u cm / %u, static %u cm / %u",
                         ld2412_state_str(data.state), data.moving_distance_cm,
                         data.moving_energy, data.static_distance_cm, data.static_energy);
            }
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(LVGL_PERIOD_MS));
    }
}
