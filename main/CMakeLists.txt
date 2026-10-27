idf_component_register(
                         SRCS "main.c" 
                              "LCD_Driver/Vernon_ST7789T/Vernon_ST7789T.c" 
                              "LCD_Driver/ST7789.c"
                              "LVGL_Driver/LVGL_Driver.c"
                              "LVGL_UI/font_digits_100.c"
                              "LVGL_UI/font_digits_85.c"
                              "LVGL_UI/font_text_20.c"
                              "RGB/RGB.c"
                              "LD2412/LD2412.c"

                         INCLUDE_DIRS 
                              "./LCD_Driver/Vernon_ST7789T" 
                              "./LCD_Driver" 
                              "./LVGL_Driver" 
                              "./LVGL_UI"
                              "./RGB" 
                              "./LD2412"
                              "."
                       )
