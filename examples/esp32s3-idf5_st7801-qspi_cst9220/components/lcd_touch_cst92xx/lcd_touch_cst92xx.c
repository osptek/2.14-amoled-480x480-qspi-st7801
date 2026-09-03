/*
 * CST9217/CST9220 touch driver for ESP-IDF.
 * Protocol aligned with ESPHome cst9220 + SensorLib TouchDrvCST92xx.
 */

#include "lcd_touch_cst92xx.h"

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "cst92xx";

#define CST92XX_REG_READ        0xD000U
#define CST92XX_REG_DEBUG_MODE  0xD101U
#define CST92XX_REG_NORMAL_MODE 0xD109U
#define CST92XX_REG_CHECKCODE   0xD1FCU
#define CST92XX_REG_RESOLUTION  0xD1F8U
#define CST92XX_REG_PROJECT_ID  0xD204U
#define CST92XX_ACK             0xABU
#define CST92XX_CHIP_CST9217    0x9217U
#define CST92XX_CHIP_CST9220    0x9220U
/* SensorLib MAX_FINGER_NUM = 2；报文长度 2*5+5 */
#define CST92XX_HW_MAX_POINTS   2U
#define CST92XX_REPORT_LEN      (CST92XX_HW_MAX_POINTS * 5U + 5U)
#define CST92XX_I2C_TIMEOUT_MS  100
#define CST92XX_REG_MODE_HS     0xD11EU
#define CST92XX_REG_MODE_STAT   0x0002U

struct lcd_touch_cst92xx {
    i2c_master_dev_handle_t i2c_dev;
    gpio_num_t rst_gpio_num;
    gpio_num_t int_gpio_num;
    uint16_t x_max;
    uint16_t y_max;
    struct {
        unsigned swap_xy : 1;
        unsigned mirror_x : 1;
        unsigned mirror_y : 1;
        unsigned int_active_high : 1;
    } flags;
};

static esp_err_t cst92xx_write(lcd_touch_cst92xx_handle_t tp, const uint8_t *data, size_t len)
{
    return i2c_master_transmit(tp->i2c_dev, data, len, CST92XX_I2C_TIMEOUT_MS);
}

static esp_err_t cst92xx_write_then_read(lcd_touch_cst92xx_handle_t tp, const uint8_t *wbuf, size_t wlen,
                                         uint8_t *rbuf, size_t rlen)
{
    return i2c_master_transmit_receive(tp->i2c_dev, wbuf, wlen, rbuf, rlen, CST92XX_I2C_TIMEOUT_MS);
}

static esp_err_t cst92xx_write_reg16(lcd_touch_cst92xx_handle_t tp, uint16_t reg)
{
    const uint8_t buf[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    return cst92xx_write(tp, buf, sizeof(buf));
}

static esp_err_t cst92xx_read_reg16(lcd_touch_cst92xx_handle_t tp, uint16_t reg, uint8_t *data, size_t len)
{
    const uint8_t buf[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    return cst92xx_write_then_read(tp, buf, sizeof(buf), data, len);
}

static esp_err_t cst92xx_hw_reset(lcd_touch_cst92xx_handle_t tp)
{
    if (tp->rst_gpio_num == GPIO_NUM_NC) {
        return ESP_OK;
    }
    /* ESPHome: high -> low(10ms) -> high */
    ESP_RETURN_ON_ERROR(gpio_set_level(tp->rst_gpio_num, 1), TAG, "reset pre-high failed");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_RETURN_ON_ERROR(gpio_set_level(tp->rst_gpio_num, 0), TAG, "reset assert failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(gpio_set_level(tp->rst_gpio_num, 1), TAG, "reset release failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t cst92xx_enter_normal_mode(lcd_touch_cst92xx_handle_t tp)
{
    /*
     * 写 0xD109 进正常扫描。本模组 checkcode=0xCBCB，切模式后不要再读 0x0002
     * 去核对，否则容易停在半命令态：单指只出首点、双指才连续。
     */
    for (int i = 0; i < 3; ++i) {
        (void)cst92xx_write_reg16(tp, CST92XX_REG_MODE_HS);
        (void)cst92xx_write_reg16(tp, CST92XX_REG_MODE_HS);
        uint8_t rb[4] = {0};
        if (cst92xx_read_reg16(tp, CST92XX_REG_MODE_STAT, rb, sizeof(rb)) == ESP_OK && rb[1] == 0x1E) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_RETURN_ON_ERROR(cst92xx_write_reg16(tp, CST92XX_REG_NORMAL_MODE), TAG, "enter normal mode failed");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "entered normal scan mode (0xD109)");
    return ESP_OK;
}

static esp_err_t cst92xx_probe(lcd_touch_cst92xx_handle_t tp)
{
    ESP_RETURN_ON_ERROR(cst92xx_hw_reset(tp), TAG, "reset failed");
    vTaskDelay(pdMS_TO_TICKS(30)); /* wait leave bootloader (ESPHome) */

    ESP_RETURN_ON_ERROR(cst92xx_write_reg16(tp, CST92XX_REG_DEBUG_MODE), TAG, "enter cmd mode failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t data[4] = {0};
    ESP_RETURN_ON_ERROR(cst92xx_read_reg16(tp, CST92XX_REG_CHECKCODE, data, sizeof(data)), TAG, "checkcode failed");
    const uint32_t checkcode = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) |
                               ((uint32_t)data[3] << 24);

    ESP_RETURN_ON_ERROR(cst92xx_read_reg16(tp, CST92XX_REG_RESOLUTION, data, sizeof(data)), TAG, "resolution failed");
    const uint16_t res_x = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    const uint16_t res_y = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    if (res_x > 0 && tp->x_max == 0) {
        tp->x_max = res_x;
    }
    if (res_y > 0 && tp->y_max == 0) {
        tp->y_max = res_y;
    }

    ESP_RETURN_ON_ERROR(cst92xx_read_reg16(tp, CST92XX_REG_PROJECT_ID, data, sizeof(data)), TAG, "project id failed");
    const uint16_t project_id = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    const uint16_t chip_id = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    ESP_RETURN_ON_FALSE(chip_id == CST92XX_CHIP_CST9217 || chip_id == CST92XX_CHIP_CST9220, ESP_ERR_NOT_SUPPORTED, TAG,
                        "unsupported chip 0x%04x", chip_id);

    ESP_LOGI(TAG, "%s ready: project=0x%04x resolution=%ux%u checkcode=0x%08lx",
             chip_id == CST92XX_CHIP_CST9220 ? "CST9220" : "CST9217", project_id, res_x, res_y,
             (unsigned long)checkcode);

    /*
     * getAttribute 会停在 0xD101 命令模式。本模组 checkcode 非 0xCACA，
     * 在该模式下常表现为：单指只出首点、双指才有后续点。
     * 二次硬复位退出命令模式，再握手进入 0xD109 正常扫描。
     */
    ESP_RETURN_ON_ERROR(cst92xx_hw_reset(tp), TAG, "reset to leave cmd mode failed");
    vTaskDelay(pdMS_TO_TICKS(30));
    return cst92xx_enter_normal_mode(tp);
}

esp_err_t lcd_touch_cst92xx_new(const lcd_touch_cst92xx_config_t *config, lcd_touch_cst92xx_handle_t *ret_touch)
{
    ESP_RETURN_ON_FALSE(config && ret_touch && config->i2c_bus, ESP_ERR_INVALID_ARG, TAG, "invalid args");

    lcd_touch_cst92xx_handle_t tp = calloc(1, sizeof(*tp));
    ESP_RETURN_ON_FALSE(tp, ESP_ERR_NO_MEM, TAG, "no mem");

    tp->rst_gpio_num = config->rst_gpio_num;
    tp->int_gpio_num = config->int_gpio_num;
    tp->x_max = config->x_max;
    tp->y_max = config->y_max;
    tp->flags.swap_xy = config->flags.swap_xy;
    tp->flags.mirror_x = config->flags.mirror_x;
    tp->flags.mirror_y = config->flags.mirror_y;

    if (tp->rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_conf = {
            .pin_bit_mask = 1ULL << tp->rst_gpio_num,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&rst_conf);
        if (err != ESP_OK) {
            free(tp);
            return err;
        }
        gpio_set_level(tp->rst_gpio_num, 1);
    }

    if (tp->int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_conf = {
            .pin_bit_mask = 1ULL << tp->int_gpio_num,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&int_conf);
        if (err != ESP_OK) {
            free(tp);
            return err;
        }
    }

    const uint32_t speed = config->scl_speed_hz ? config->scl_speed_hz : 400000;
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_TOUCH_CST92XX_I2C_ADDRESS,
        .scl_speed_hz = speed,
    };
    esp_err_t err = i2c_master_bus_add_device(config->i2c_bus, &dev_cfg, &tp->i2c_dev);
    if (err != ESP_OK) {
        free(tp);
        return err;
    }

    err = cst92xx_probe(tp);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(tp->i2c_dev);
        free(tp);
        return err;
    }

    *ret_touch = tp;
    return ESP_OK;
}

bool lcd_touch_cst92xx_intr_asserted(lcd_touch_cst92xx_handle_t touch)
{
    if (!touch || touch->int_gpio_num == GPIO_NUM_NC) {
        return true;
    }
    const int level = gpio_get_level(touch->int_gpio_num);
    return touch->flags.int_active_high ? (level != 0) : (level == 0);
}

esp_err_t lcd_touch_cst92xx_read(lcd_touch_cst92xx_handle_t touch, uint16_t *x, uint16_t *y, uint16_t *strength,
                                 uint8_t *point_count, uint8_t max_points, lcd_touch_cst92xx_status_t *status)
{
    ESP_RETURN_ON_FALSE(touch && x && y && point_count && max_points > 0, ESP_ERR_INVALID_ARG, TAG, "invalid args");
    *point_count = 0;
    if (status) {
        *status = LCD_TOUCH_CST92XX_IDLE;
    }

    uint8_t buffer[CST92XX_REPORT_LEN] = {0};
    const uint8_t reg[2] = {(uint8_t)(CST92XX_REG_READ >> 8), (uint8_t)CST92XX_REG_READ};
    esp_err_t err = cst92xx_write_then_read(touch, reg, sizeof(reg), buffer, sizeof(buffer));
    if (err != ESP_OK) {
        /* 芯片刚 ACK 完或正在组下一帧时会 NACK，不当成致命错误连打 */
        ESP_LOGD(TAG, "report read failed: %s", esp_err_to_name(err));
        return ESP_OK;
    }

    bool all_zero = true;
    for (size_t i = 0; i < sizeof(buffer); ++i) {
        if (buffer[i] != 0) {
            all_zero = false;
            break;
        }
    }

    /* 空邮箱不要 ACK，否则会打乱下一帧准备 */
    if (all_zero) {
        return ESP_OK;
    }

    /* CST9220：读完必须立刻 ACK，芯片才放出下一帧。解析放在 ACK 之后。 */
    const uint8_t ack[3] = {(uint8_t)(CST92XX_REG_READ >> 8), (uint8_t)CST92XX_REG_READ, CST92XX_ACK};
    err = cst92xx_write(touch, ack, sizeof(ack));
    if (err != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
        err = cst92xx_write(touch, ack, sizeof(ack));
    }
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "ack 0xAB failed: %s", esp_err_to_name(err));
        return ESP_OK;
    }
    if (status) {
        *status = LCD_TOUCH_CST92XX_DROPPED;
    }

    ESP_LOGD(TAG, "raw %02X%02X%02X%02X%02X %02X %02X |%02X|", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4],
             buffer[5], buffer[6], buffer[7]);

    if (buffer[0] == CST92XX_ACK || buffer[6] != CST92XX_ACK || buffer[0] == 0x00) {
        return ESP_OK;
    }

    /* SensorLib：buffer[4] bit7 为盖屏手势，本帧无触点 */
    if ((buffer[4] & 0xF0) != 0 && (buffer[4] >> 7) == 0x01) {
        return ESP_OK;
    }

    if (status) {
        *status = LCD_TOUCH_CST92XX_FRAME;
    }

    uint8_t num = buffer[5] & 0x7F;
    if (num == 0) {
        return ESP_OK;
    }
    if (num > CST92XX_HW_MAX_POINTS) {
        num = CST92XX_HW_MAX_POINTS;
    }

    uint8_t out = 0;
    for (uint8_t i = 0; i < num && out < max_points; ++i) {
        const uint8_t *pdat = buffer + (size_t)i * 5U + (i == 0 ? 0U : 2U);
        const uint8_t event = pdat[0] & 0x0F;
        /*
         * 本模组单指常用非 0x06 事件。只丢掉明确抬起 0x00/0x05，
         * 其余带坐标的帧都收，否则单指被滤掉、双指才有反应。
         */
        if (event == 0x00U || event == 0x05U) {
            continue;
        }

        uint16_t px = ((uint16_t)pdat[1] << 4) | (pdat[3] >> 4);
        uint16_t py = ((uint16_t)pdat[2] << 4) | (pdat[3] & 0x0F);

        if (touch->flags.mirror_x && touch->x_max > 0) {
            px = (px < touch->x_max) ? (touch->x_max - px) : 0;
        }
        if (touch->flags.mirror_y && touch->y_max > 0) {
            py = (py < touch->y_max) ? (touch->y_max - py) : 0;
        }
        if (touch->flags.swap_xy) {
            const uint16_t tmp = px;
            px = py;
            py = tmp;
        }

        x[out] = px;
        y[out] = py;
        if (strength) {
            strength[out] = 1;
        }
        ++out;
    }

    *point_count = out;
    return ESP_OK;
}
