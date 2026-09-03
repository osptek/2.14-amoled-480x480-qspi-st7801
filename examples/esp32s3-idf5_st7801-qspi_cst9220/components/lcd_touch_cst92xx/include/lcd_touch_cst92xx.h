#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file lcd_touch_cst92xx.h
 * @brief CST9217/CST9220 触摸驱动（协议对齐 SensorLib TouchDrvCST92xx）
 *
 * 说明：原版 TouchDrvCST92xx 依赖完整 SensorLib，且 getAttribute() 要求
 * checkcode 高 16 位为 0xCACA。本模组实测为 0xCBCB，直接套用会初始化失败。
 * 因此将协议移植为独立 ESP-IDF C 驱动；原版头文件见 ../ref/TouchDrvCST92xx.h
 */

#define LCD_TOUCH_CST92XX_I2C_ADDRESS 0x5A
#define LCD_TOUCH_CST92XX_MAX_POINTS  2

typedef struct lcd_touch_cst92xx *lcd_touch_cst92xx_handle_t;

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    uint32_t scl_speed_hz;           /* 0 = default 400kHz */
    gpio_num_t rst_gpio_num;         /* GPIO_NUM_NC if unused */
    gpio_num_t int_gpio_num;         /* GPIO_NUM_NC if unused */
    uint16_t x_max;
    uint16_t y_max;
    struct {
        unsigned swap_xy : 1;
        unsigned mirror_x : 1;
        unsigned mirror_y : 1;
    } flags;
} lcd_touch_cst92xx_config_t;

typedef enum {
    LCD_TOUCH_CST92XX_IDLE = 0, /**< 邮箱空（全 0），未写 ACK */
    LCD_TOUCH_CST92XX_FRAME,    /**< 已 ACK 且报文可解析；point_count=0 表示抬起 */
    LCD_TOUCH_CST92XX_DROPPED,  /**< 已 ACK 但头字节/手势无效，忽略本帧 */
} lcd_touch_cst92xx_status_t;

esp_err_t lcd_touch_cst92xx_new(const lcd_touch_cst92xx_config_t *config, lcd_touch_cst92xx_handle_t *ret_touch);

/**
 * 读 0xD000 邮箱。非空帧必须立刻写 ACK 0xAB，否则芯片不能准备下一帧。
 * status 可为 NULL。
 */
esp_err_t lcd_touch_cst92xx_read(lcd_touch_cst92xx_handle_t touch, uint16_t *x, uint16_t *y, uint16_t *strength,
                                 uint8_t *point_count, uint8_t max_points, lcd_touch_cst92xx_status_t *status);

bool lcd_touch_cst92xx_intr_asserted(lcd_touch_cst92xx_handle_t touch);

#ifdef __cplusplus
}
#endif
