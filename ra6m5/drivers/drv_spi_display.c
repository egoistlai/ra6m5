/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "drv_spi_display.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "lvgl.h"

extern lv_disp_drv_t *g_disp_drv_ptr;

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define LCD_DC_PIN BSP_IO_PORT_01_PIN_04
#define LCD_RESET_PIN BSP_IO_PORT_01_PIN_05
#define LCD_PWM_PIN BSP_IO_PORT_06_PIN_08

#define SPI_SEND_DATA BSP_IO_LEVEL_HIGH
#define SPI_SEND_CMD BSP_IO_LEVEL_LOW

/* ST7796S部分寄存器定义 */
#define LCD_DISPLAY_CMD_RAMCTRL 0xb0 // RAM Control
#define LCD_DISPLAY_CMD_CASET 0x2a   // Column address set
#define LCD_DISPLAY_CMD_RASET 0x2b   // Row address set
#define LCD_DISPLAY_CMD_RAMWR 0x2c   // Memory write

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static void spi1_wait_for_tx(void);
static void spi_display_init(void);

static fsp_err_t spi_send_data_cmd(uint8_t *uc_data, bsp_io_level_t uc_cmd, uint32_t len);
static fsp_err_t spi_display_backlight_opt(bsp_io_level_t opt);
static fsp_err_t spi_display_reset(void);

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
/* 替换原来的 volatile flag 为信号量 */
static SemaphoreHandle_t g_spi_tx_sem = NULL;

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

fsp_err_t drv_spi_display_init(void)
{
    fsp_err_t err;

    /* 1. 创建二值信号量 */
    g_spi_tx_sem = xSemaphoreCreateBinary();
    if (g_spi_tx_sem == NULL)
    {
        __BKPT();
    }

    /* 初始化SPI驱动 */
    err = g_spi1.p_api->open(&g_spi1_ctrl, &g_spi1_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("%s %d\r\n", __FUNCTION__, __LINE__);
        return err;
    }

    spi_display_init();

    return err;
}

void spi_display_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint8_t caset[4];
    uint8_t raset[4];

    caset[0] = (uint8_t)(x1 >> 8) & 0xFF;
    caset[1] = (uint8_t)(x1 & 0xff);
    caset[2] = (uint8_t)(x2 >> 8) & 0xFF;
    caset[3] = (uint8_t)(x2 & 0xff);

    raset[0] = (uint8_t)(y1 >> 8) & 0xFF;
    raset[1] = (uint8_t)(y1 & 0xff);
    raset[2] = (uint8_t)(y2 >> 8) & 0xFF;
    raset[3] = (uint8_t)(y2 & 0xff);

    spi_send_data_cmd((uint8_t[]){LCD_DISPLAY_CMD_CASET}, SPI_SEND_CMD, 1); // Horiz
    spi_send_data_cmd(caset, SPI_SEND_DATA, 4);
    spi_send_data_cmd((uint8_t[]){LCD_DISPLAY_CMD_RASET}, SPI_SEND_CMD, 1); // Vert
    spi_send_data_cmd(raset, SPI_SEND_DATA, 4);
    spi_send_data_cmd((uint8_t[]){LCD_DISPLAY_CMD_RAMWR}, SPI_SEND_CMD, 1); // Memory write
}

fsp_err_t drv_spi_display_flush_data(uint8_t *data, uint32_t len)
{
    // 拉高 DC 引脚，表示发送的是图像数据
    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, LCD_DC_PIN, SPI_SEND_DATA);

    // 启动 SPI 传输。因为你配置了 DTC/DMA，这一步会立刻返回，底层硬件会在后台自动搬运数据
    return g_spi1.p_api->write(g_spi1.p_ctrl, data, len, SPI_BIT_WIDTH_8_BITS);
}

void spi1_callback(spi_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        /* 传输完成，释放信号量 */
        if (g_spi_tx_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_spi_tx_sem, &xHigherPriorityTaskWoken);
        }
        if (g_disp_drv_ptr != NULL)
        {
            lv_disp_flush_ready(g_disp_drv_ptr);
        }
    }
    else
    {
        /* 错误处理：通常也可以释放信号量让任务解除阻塞，或者设置错误标志 */
        if (g_spi_tx_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_spi_tx_sem, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
static void spi1_wait_for_tx(void)
{
    /* 阻塞等待信号量，使用 portMAX_DELAY 确保一直等到传输完成 */
    /* 也可以设置一个具体的超时时间，例如 pdMS_TO_TICKS(100) */
    if (g_spi_tx_sem != NULL)
    {
        xSemaphoreTake(g_spi_tx_sem, portMAX_DELAY);
    }
}

static void spi_display_init(void)
{
    spi_display_reset();
    spi_display_backlight_opt(BSP_IO_LEVEL_HIGH); // backlight on

    /* 这里的延时使用 vTaskDelay 替换 R_BSP_SoftwareDelay 会更高效，
       但初始化通常只执行一次，且在 main 中执行，保持 R_BSP 也无妨。
       为了展示 RTOS 特性，建议后续初始化放到任务中。
    */

    spi_send_data_cmd((uint8_t[]){0x11}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x00}, SPI_SEND_DATA, 1);
    R_BSP_SoftwareDelay(120, BSP_DELAY_UNITS_MILLISECONDS);

    spi_send_data_cmd((uint8_t[]){0xf0}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0xc3}, SPI_SEND_DATA, 1);
    spi_send_data_cmd((uint8_t[]){0xf0}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x96}, SPI_SEND_DATA, 1);
    spi_send_data_cmd((uint8_t[]){0x36}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x48}, SPI_SEND_DATA, 1); // RGB
    spi_send_data_cmd((uint8_t[]){0xb4}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x01}, SPI_SEND_DATA, 1);
    spi_send_data_cmd((uint8_t[]){0xb7}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0xc6}, SPI_SEND_DATA, 1);

    spi_send_data_cmd((uint8_t[]){0xe8}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33}, SPI_SEND_DATA, 8);

    spi_send_data_cmd((uint8_t[]){0xc1}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x06}, SPI_SEND_DATA, 1);
    spi_send_data_cmd((uint8_t[]){0xc2}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0xa7}, SPI_SEND_DATA, 1);
    spi_send_data_cmd((uint8_t[]){0xc5}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x18}, SPI_SEND_DATA, 1);

    spi_send_data_cmd((uint8_t[]){0xe0}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B}, SPI_SEND_DATA, 14);

    spi_send_data_cmd((uint8_t[]){0xe1}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0xF0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2D, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B}, SPI_SEND_DATA, 14);

    spi_send_data_cmd((uint8_t[]){0xf0}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x3c}, SPI_SEND_DATA, 1);
    spi_send_data_cmd((uint8_t[]){0xf0}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x69}, SPI_SEND_DATA, 1);
    spi_send_data_cmd((uint8_t[]){0x3a}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x55}, SPI_SEND_DATA, 1);
    R_BSP_SoftwareDelay(120, BSP_DELAY_UNITS_MILLISECONDS);

    spi_send_data_cmd((uint8_t[]){0x21}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x29}, SPI_SEND_CMD, 1);

    /*rotation*/
    spi_send_data_cmd((uint8_t[]){0x36}, SPI_SEND_CMD, 1);
    spi_send_data_cmd((uint8_t[]){0x48}, SPI_SEND_DATA, 1); // 0
}

static fsp_err_t spi_send_data_cmd(uint8_t *uc_data, bsp_io_level_t uc_cmd, uint32_t len)
{
    fsp_err_t err = FSP_SUCCESS; // Error status

    /* Master send data to device */
    err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl, LCD_DC_PIN, uc_cmd);
    if (FSP_SUCCESS != err)
    {
        printf("%s %d\r\n", __FUNCTION__, __LINE__);
        return err;
    }

    err = g_spi1.p_api->write(g_spi1.p_ctrl, uc_data, len, SPI_BIT_WIDTH_8_BITS);
    if (FSP_SUCCESS != err)
    {
        printf("%s %d\r\n", __FUNCTION__, __LINE__);
        return err;
    }

    spi1_wait_for_tx();

    return err;
}

static fsp_err_t spi_display_backlight_opt(bsp_io_level_t opt)
{
    fsp_err_t err = FSP_SUCCESS; // Error status

    g_ioport.p_api->pinWrite((ioport_ctrl_t *const)&g_ioport.p_ctrl, LCD_PWM_PIN, opt);
    return err;
}

static fsp_err_t spi_display_reset(void)
{
    fsp_err_t err = FSP_SUCCESS; // Error status

    g_ioport.p_api->pinWrite((ioport_ctrl_t *const)&g_ioport.p_ctrl, LCD_RESET_PIN, BSP_IO_LEVEL_LOW);
    R_BSP_SoftwareDelay(120, BSP_DELAY_UNITS_MILLISECONDS);
    g_ioport.p_api->pinWrite((ioport_ctrl_t *const)&g_ioport.p_ctrl, LCD_RESET_PIN, BSP_IO_LEVEL_HIGH);
    R_BSP_SoftwareDelay(120, BSP_DELAY_UNITS_MILLISECONDS);

    return err;
}