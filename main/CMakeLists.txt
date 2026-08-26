idf_component_register(
                         SRCS "main.c" 
                              "LCD_Driver/Vernon_ST7789T/Vernon_ST7789T.c" 
                              "LCD_Driver/ST7789.c"
                              "LVGL_Driver/LVGL_Driver.c"
                              "LVGL_UI/LVGL_Example.c"
                              "LVGL_UI/font_digits_100.c"
                              "LVGL_UI/font_text_20.c"
                              "SD_Card/SD_SPI.c"
                              "RGB/RGB.c"
                              "Wireless/Wireless.c"
                              "LD2412/LD2412.c"

                         INCLUDE_DIRS 
                              "./LCD_Driver/Vernon_ST7789T" 
                              "./LCD_Driver" 
                              "./LVGL_Driver" 
                              "./LVGL_UI" 
                              "./SD_Card"
                              "./RGB" 
                              "./Wireless"
                              "./LD2412"
                              "."
                       )
