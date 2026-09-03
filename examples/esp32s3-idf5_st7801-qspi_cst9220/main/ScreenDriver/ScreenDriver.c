#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lcd_st7801.h"
#include "lcd_touch_cst92xx.h"

#include "ScreenDriver.h"      //加载pin map

static const char *TAG = "driver";
static SemaphoreHandle_t lvgl_mux = NULL;
#if EXAMPLE_ENABLE_TE
static SemaphoreHandle_t te_sem = NULL;
#endif
static SemaphoreHandle_t touch_sem = NULL;
static lcd_touch_cst92xx_handle_t tp = NULL;
static lv_indev_t *s_indev = NULL;
static volatile TaskHandle_t s_lvgl_task = NULL;
static portMUX_TYPE s_touch_mux = portMUX_INITIALIZER_UNLOCKED;
static int16_t s_touch_last_x;
static int16_t s_touch_last_y;
static bool s_touch_pressed;
static int64_t s_touch_last_point_us;

#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 8)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (8 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2
/* CST9220 邮箱任务必须高于 LVGL，否则 ACK 会被刷屏拖住 */
#define CST9220_MAILBOX_TASK_PRIORITY  5
#define CST9220_MAILBOX_STACK_SIZE     (4 * 1024)
/* 正常扫描约 10ms/帧；2~3 帧无新邮箱视为抬起 */
#define CST9220_RELEASE_US             35000

/* ST7801N QSPI 初始化 */
static const st7801_lcd_init_cmd_t lcd_init_cmds[] =
{
    {0x11, (uint8_t []){0x00}, 0, 200},                          // Sleep Out
    {0x2A, (uint8_t []){0x00, 0x00, 0x01, 0xDF}, 4, 0},          // Column: 0~479
    {0x2B, (uint8_t []){0x00, 0x00, 0x01, 0xDF}, 4, 0},          // Row: 0~479
    {0x53, (uint8_t []){0x20}, 1, 0},
    // {0x36, (uint8_t []){0x00}, 1, 0},
    {0x51, (uint8_t []){0x9F}, 1, 0},                            // 亮度 0x00~0xFF
    {0x3A, (uint8_t []){0x55}, 1, 10},                            // RGB565

    #if EXAMPLE_ENABLE_TE
    {0x35, (uint8_t []){0x01}, 1, 10},                            // TE 使能
#else
    {0x35, (uint8_t []){0x00}, 1, 0},                             // TE 关闭（保留命令，便于对照）
#endif

    {0x29, (uint8_t []){0x00}, 0, 200},                            // Display On
};

#if EXAMPLE_ENABLE_TE
/*帧同步*/
static void IRAM_ATTR te_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(te_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
#endif

/* CST9220 INT：每准备好一帧邮箱拉低一次。ISR 只唤醒邮箱任务，立刻 Read+ACK。 */
static void IRAM_ATTR touch_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void cst9220_notify_lvgl(void)
{
    TaskHandle_t lvgl_task = s_lvgl_task;
    if (lvgl_task) {
        xTaskNotifyGive(lvgl_task);
    }
}

static void cst9220_store_sample(int16_t x, int16_t y, bool pressed)
{
    taskENTER_CRITICAL(&s_touch_mux);
    s_touch_last_x = x;
    s_touch_last_y = y;
    s_touch_pressed = pressed;
    if (pressed) {
        s_touch_last_point_us = esp_timer_get_time();
    }
    taskEXIT_CRITICAL(&s_touch_mux);
}

/* CST9220：一次 INT 对应一帧邮箱。ACK 后立刻再读会撞上芯片组包，I2C 直接 NACK。 */
static void cst9220_mailbox_task(void *arg)
{
    lcd_touch_cst92xx_handle_t touch = (lcd_touch_cst92xx_handle_t)arg;
    ESP_LOGI(TAG, "CST9220 mailbox task started");

    for (;;) {
        (void)xSemaphoreTake(touch_sem, portMAX_DELAY);

        uint16_t touch_x[LCD_TOUCH_CST92XX_MAX_POINTS] = {0};
        uint16_t touch_y[LCD_TOUCH_CST92XX_MAX_POINTS] = {0};
        uint8_t point_count = 0;
        lcd_touch_cst92xx_status_t st = LCD_TOUCH_CST92XX_IDLE;

        if (lcd_touch_cst92xx_read(touch, touch_x, touch_y, NULL, &point_count, LCD_TOUCH_CST92XX_MAX_POINTS, &st) !=
            ESP_OK) {
            continue;
        }
        if (st != LCD_TOUCH_CST92XX_FRAME) {
            continue;
        }

        /* 无触点只表示本帧没解出坐标，不要立刻当抬起；单指会被误杀 */
        if (point_count > 0) {
            cst9220_store_sample((int16_t)touch_x[0], (int16_t)touch_y[0], true);
            cst9220_notify_lvgl();
        }
    }
}

static void example_touch_isr_init(void)
{
    touch_sem = xSemaphoreCreateCounting(8, 0);
    assert(touch_sem);

    /* 驱动已把 INT 配成输入；TE 初始化已经装过 ISR service */
    ESP_ERROR_CHECK(gpio_set_intr_type(TOUCH_INT, GPIO_INTR_NEGEDGE));
#if !EXAMPLE_ENABLE_TE
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
#endif
    ESP_ERROR_CHECK(gpio_isr_handler_add(TOUCH_INT, touch_isr_handler, NULL));
    ESP_LOGI(TAG, "Touch INT enabled on GPIO%d", (int)TOUCH_INT);

    const BaseType_t ok = xTaskCreate(cst9220_mailbox_task, "cst9220", CST9220_MAILBOX_STACK_SIZE, tp,
                                      CST9220_MAILBOX_TASK_PRIORITY, NULL);
    assert(ok == pdPASS);
}
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    const int offsetx1 = area->x1 + EXAMPLE_LCD_X_GAP;
    const int offsetx2 = area->x2 + EXAMPLE_LCD_X_GAP;
    const int offsety1 = area->y1 + EXAMPLE_LCD_Y_GAP;
    const int offsety2 = area->y2 + EXAMPLE_LCD_Y_GAP;

#if EXAMPLE_ENABLE_TE
    if (xSemaphoreTake(te_sem, pdMS_TO_TICKS(60)) != pdTRUE) {
        ESP_LOGW(TAG, "TE timeout"); //等待60ms如果未触发TE就强制刷新
    }
#endif
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_lvgl_update_cb(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_90:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_180:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;
    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;
    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;

    int16_t x;
    int16_t y;
    bool pressed;
    int64_t last_us;
    taskENTER_CRITICAL(&s_touch_mux);
    x = s_touch_last_x;
    y = s_touch_last_y;
    pressed = s_touch_pressed;
    last_us = s_touch_last_point_us;
    taskEXIT_CRITICAL(&s_touch_mux);

    data->point.x = x;
    data->point.y = y;
    if (!pressed) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    /* 邮箱约 10ms/帧；超过 2~3 帧没有新点则抬起，不再用 120ms 粘滞 */
    if ((esp_timer_get_time() - last_us) > CST9220_RELEASE_US) {
        cst9220_store_sample(x, y, false);
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }
    data->state = LV_INDEV_STATE_PRESSED;
}

static void example_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static bool example_lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        if (example_lvgl_lock(-1))
        {
            if (s_indev && s_indev->driver && s_indev->driver->read_timer) {
                lv_timer_ready(s_indev->driver->read_timer);
            }
            task_delay_ms = lv_timer_handler();
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        /* CST9220 邮箱任务 ACK 后 notify，立即消费最新点，不睡满 500ms */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(task_delay_ms));
    }
}

void SetUP_Screen(void)//点屏主要代码
{
#if EXAMPLE_ENABLE_TE
    te_sem = xSemaphoreCreateBinary();
    assert(te_sem != NULL && "Failed to create TE semaphore");

    /*初始化TE的GPIO*/
    gpio_config_t te_conf = {
        .pin_bit_mask = (1ULL << AMOLED_TE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&te_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(AMOLED_TE, te_isr_handler, NULL));
    xSemaphoreTake(te_sem, 0);
    ESP_LOGI(TAG, "TE sync enabled");
#else
    ESP_LOGI(TAG, "TE sync disabled");
#endif

#if AMOLED_POWER
    /*拉高屏幕电源使能*/
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << AMOLED_PWREN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "LCD PWREN enabled");
#else
    ESP_LOGI(TAG, "LCD PWREN disabled");
#endif

    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;

    /*初始化QSPI*/
    const spi_bus_config_t buscfg = ST7801_PANEL_BUS_QSPI_CONFIG(AMOLED_CLK,
                                                                 AMOLED_D0,
                                                                 AMOLED_D1,
                                                                 AMOLED_D2,
                                                                 AMOLED_D3,
                                                                 EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = ST7801_PANEL_IO_QSPI_CONFIG(AMOLED_CS,
                                                                          example_notify_lvgl_flush_ready,
                                                                          &disp_drv);
    io_config.pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ; /* 覆盖组件内置 80MHz */
    ESP_LOGI(TAG, "LCD QSPI pclk=%lu Hz", (unsigned long)io_config.pclk_hz);
    st7801_vendor_config_t vendor_config =
    {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags =
        {
            .use_qspi_interface = 1,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config =
    {
        .reset_gpio_num = AMOLED_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(TAG, "Install ST7801 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7801(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    i2c_master_bus_handle_t i2c_bus = NULL;
    const i2c_master_bus_config_t i2c_bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TOUCH_HOST,
        .scl_io_num = SCL,
        .sda_io_num = SDA,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));

    const lcd_touch_cst92xx_config_t tp_cfg = {
        .i2c_bus = i2c_bus,
        .scl_speed_hz = 400 * 1000,
        .rst_gpio_num = TOUCH_RST,
        .int_gpio_num = TOUCH_INT,
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    ESP_LOGI(TAG, "Install CST9220 touch driver");
    ESP_ERROR_CHECK(lcd_touch_cst92xx_new(&tp_cfg, &tp));
    example_touch_isr_init();

    lv_init();


    const size_t draw_buf_pixels = EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT;
    const size_t draw_buf_sz = draw_buf_pixels * sizeof(lv_color_t);
    lv_color_t *buf1 = heap_caps_malloc(draw_buf_sz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_color_t *buf2 = heap_caps_malloc(draw_buf_sz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf1 && buf2);
    ESP_LOGI(TAG, "LVGL draw buf %u bytes x2 (internal DMA)", (unsigned)draw_buf_sz);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, draw_buf_pixels);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.rounder_cb = example_lvgl_rounder_cb;
    disp_drv.drv_update_cb = example_lvgl_update_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args =
    {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;
    s_indev = lv_indev_drv_register(&indev_drv);
    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
    //disp_drv.rotated = LV_DISP_ROT_90;

    /*创建LVGL任务*/
    TaskHandle_t lvgl_task = NULL;
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, &lvgl_task);
    s_lvgl_task = lvgl_task;

    ESP_LOGI(TAG, "Display UI");
    if (example_lvgl_lock(-1))
    {
        lv_demo_music();
        // lv_demo_widgets();
        example_lvgl_unlock();
    }
}
