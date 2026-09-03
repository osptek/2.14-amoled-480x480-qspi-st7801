file(GLOB_RECURSE srcs *.c
                    *.cpp
                    ScreenDriver/*.c
                    )
 
set(include_dirs 
    . 
    ScreenDriver
    )
 
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES driver esp_lcd esp_timer esp_lcd_st7801 lcd_touch_cst92xx lvgl__lvgl
    )

add_compile_definitions(LV_LVGL_H_INCLUDE_SIMPLE)
