#include "esp_mac.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "ST7789.h"
#include "SD_SPI.h"
#include "RGB.h"
#include "Wireless.h"
#include "LVGL_Example.h"
#include "LD2412/LD2412.h"

#include "lvgl.h"

#define TAG "MAIN"


void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize LD2412
    ld2412_init();
    // ld2412_enable_config();

    // char fw[32] = {0};
    // if (ld2412_read_firmware_version(fw, sizeof(fw)) == ESP_OK) {
    //     ESP_LOGI(TAG, "LD2412 Firmware: %s", fw);
    // } else {
    //     ESP_LOGE(TAG, "Failed to read firmware version");
    //     snprintf(fw, sizeof(fw), "Read failed");
    // }
    // ld2412_end_config();

    // Initialize peripherals
    //Wireless_Init();
    Flash_Searching();
    RGB_Init();
    RGB_Example();
    SD_Init();
    LCD_Init();
    BK_Light(50);
    LVGL_Init();


    // Create LVGL labels
    lv_obj_t *dist_label = lv_label_create(lv_scr_act());
    lv_obj_set_width(dist_label, 200);  // Fixed width to prevent shifting
    lv_label_set_text(dist_label, "Distance: --.- m");
    lv_obj_align(dist_label, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *signal_label = lv_label_create(lv_scr_act());
    lv_obj_set_width(signal_label, 200);
    lv_label_set_text(signal_label, "Signal: ---");
    lv_obj_align(signal_label, LV_ALIGN_CENTER, 0, 20);

    // Start radar data stream
    //ld2412_start_stream();

    // Main loop
       // Variables to hold last valid values
    float last_distance_m = 0.0f;
    uint8_t last_signal = 0;
    ld2412_frame_t frame;
    ld2412_enable_configuration(UART_NUM_1);
    ld2412_set_output_mode(UART_NUM_1, true);  // Set to basic mode
    ld2412_exit_configuration(UART_NUM_1);
    // Main loop
    while (1) {
        
        frame.targets[0].signal_strength = -1;
        if (ld2412_read_frame(&frame) == ESP_OK) {
            uint16_t distance = frame.targets[0].distance_mm;
            uint8_t signal = frame.targets[0].signal_strength;

            if (signal >= 20) {
                last_distance_m = distance / 100.0f;
                last_signal = signal;
            }

            // Update display with last known good values
            char dist_str[32];
            snprintf(dist_str, sizeof(dist_str), "Distance: %.1f m", last_distance_m);
            lv_label_set_text(dist_label, dist_str);

            char signal_str[32];
            snprintf(signal_str, sizeof(signal_str), "Signal: %3d/%3d", last_signal, signal);
            lv_label_set_text(signal_label, signal_str);
            ld2412_parse_frame(&frame);
            //ESP_LOGI(TAG, "Distance: %.1f m, Signal: %d", last_distance_m, last_signal);
        } else {
            lv_label_set_text(signal_label, "Get bad frame");
        }
        
        // ESP_LOGI(TAG, "Raw Data:");
        // ESP_LOG_BUFFER_HEX(TAG, frame.buf, sizeof(frame.buf));
        // ESP_LOGI(TAG, "Frame start index: %d", frame.start);
        // ESP_LOGI(TAG, "Frame end index: %d", frame.endData);

       
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

}
