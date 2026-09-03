idf_component_register(SRCS "esp_lcd_st7801_spi.c" "esp_lcd_st7801_mipi.c" "esp_lcd_st7801.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "priv_include"
                       REQUIRES "esp_lcd" "driver")


