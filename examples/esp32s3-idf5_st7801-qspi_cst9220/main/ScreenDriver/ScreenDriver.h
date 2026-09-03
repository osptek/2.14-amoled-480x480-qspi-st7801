#ifndef SCREEN_DRIVER_H
#define SCREEN_DRIVER_H

#include <stdbool.h>
#include <stdio.h>

#define LCD_HOST                       SPI2_HOST
#define TOUCH_HOST                     I2C_NUM_0
#define LCD_BIT_PER_PIXEL              (16)

/* 显示：ST7801；触摸：CST9220 */
#define EXAMPLE_LCD_H_RES              480
#define EXAMPLE_LCD_V_RES              480

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (80 * 1000 * 1000)

/* 刷新偏移 */
#define EXAMPLE_LCD_X_GAP              0
#define EXAMPLE_LCD_Y_GAP              0

#define EXAMPLE_ENABLE_TE              1
#define AMOLED_POWER                   0

/* 点屏引脚分配，在后面的括号改成实际的 */
#define SDA               (GPIO_NUM_39)
#define SCL               (GPIO_NUM_40)
#define TOUCH_RST         (GPIO_NUM_41)
#define TOUCH_INT         (GPIO_NUM_42)
#define AMOLED_TE         (GPIO_NUM_11)
#define AMOLED_RST        (GPIO_NUM_10)
#define AMOLED_CS         (GPIO_NUM_9)
#define AMOLED_CLK        (GPIO_NUM_12)
#define AMOLED_D0         (GPIO_NUM_13)
#define AMOLED_D1         (GPIO_NUM_14)
#define AMOLED_D2         (GPIO_NUM_15)
#define AMOLED_D3         (GPIO_NUM_16)
#define AMOLED_PWREN      (GPIO_NUM_1)//实际上转接板没有用这个IO，我给随便写的

void SetUP_Screen(void);

#endif
