/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "drv_i2c_touchpad.h"
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
/* 请根据实际电路修改引脚定义 */
#define TOUCH_RESET_PIN BSP_IO_PORT_04_PIN_03
#define TOUCH_INT_PIN BSP_IO_PORT_04_PIN_08

/* FT6336U I2C Slave Address */
#define FT6336U_SLAVE_ADDR 0x38

/* FT6336U 寄存器定义 */
#define FT_DEV_MODE_REG 0x00      // 设备模式，0x00为正常工作模式
#define FT_GEST_ID_REG 0x01       // 手势ID
#define FT_TD_STATUS_REG 0x02     // 触摸状态，有效触点数量
#define FT_TP1_REG 0x03           // 第一个触摸点数据起始地址
#define FT_TP2_REG 0x09           // 第二个触摸点数据起始地址
#define FT_ID_G_LIB_VERSION 0xA1  // 库版本
#define FT_ID_G_MODE 0xA4         // 中断模式
#define FT_ID_G_THGROUP 0x80      // 触摸阈值
#define FT_ID_G_PERIODACTIVE 0x88 // 激活状态周期
#define FT_ID_G_CIPHER 0xA3       // 芯片ID寄存器

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/** 用于存放每一个触控点的id，坐标，大小 **/
typedef struct
{
    uint8_t id;
    uint16_t x;
    uint16_t y;
    uint16_t event; // 触摸事件类型
} tp_point_info_t;

/** 类结构体 **/
typedef struct
{
    uint8_t tp_dev_addr;
    uint16_t height;
    uint16_t width;
    tp_rotation_t rotation;
    tp_point_info_t points_info[TOUCH_POINT_TOTAL]; // 用于存储触控点的坐标
} tp_drv_t;

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/
static fsp_err_t i2c2_wait_for_tx(void);
static fsp_err_t i2c2_wait_for_rx(void);

static fsp_err_t ft6336u_write_reg(uint8_t reg, uint8_t *buf, uint8_t len);
static fsp_err_t ft6336u_read_reg(uint8_t reg, uint8_t *buf, uint8_t len);
static void ft6336u_reset(void);

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
static tp_drv_t g_tp_drv;

/* 替换 volatile bool 为信号量 */
static SemaphoreHandle_t g_i2c_tx_sem = NULL;
static SemaphoreHandle_t g_i2c_rx_sem = NULL;

/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

/* 测试函数，保持原有逻辑 */
fsp_err_t drv_i2c_touchpad_test(void)
{
    fsp_err_t err;
    uint16_t x = 0, y = 0;

    err = touchpad_is_touched();

    if (FSP_SUCCESS == err)
    {
        for (int i = 0; i < TOUCH_POINT_TOTAL; ++i)
        {
            touchpad_get_pos(&x, &y, i);
            if (x != 0 || y != 0)
            {
                //printf("No: %d, touched x: %d, touched y: %d\r\n", i, x, y);
            }
        }
    }
    return err;
}

fsp_err_t drv_i2c_touchpad_init(void)
{
    fsp_err_t err;
    uint8_t buf[2];

    /* 1. 创建信号量 */
    g_i2c_tx_sem = xSemaphoreCreateBinary();
    g_i2c_rx_sem = xSemaphoreCreateBinary();
    if ((g_i2c_tx_sem == NULL) || (g_i2c_rx_sem == NULL))
    {
        __BKPT();
    }

    /* 2. 初始化I2C驱动 */
    err = g_i2c_master2.p_api->open(g_i2c_master2.p_ctrl, g_i2c_master2.p_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("I2C Open Failed: %s %d\r\n", __FUNCTION__, __LINE__);
        return err;
    }

    err = g_i2c_master2.p_api->slaveAddressSet(g_i2c_master2.p_ctrl, FT6336U_SLAVE_ADDR, I2C_MASTER_ADDR_MODE_7BIT);
    if (FSP_SUCCESS != err)
    {
        printf("Address Set Failed\r\n");
        return err;
    }

    ft6336u_reset();
    touchpad_set_rotation(TP_ROT_NONE);
    g_tp_drv.width = 320;
    g_tp_drv.height = 480;

    err = ft6336u_read_reg(FT_ID_G_CIPHER, buf, 1);
    if (FSP_SUCCESS == err)
    {
        printf("FT6336U Chip ID: 0x%02x\r\n", buf[0]);
    }
    else
    {
        printf("FT6336U Read ID Failed\r\n");
        return err;
    }

    buf[0] = 0x00;
    ft6336u_write_reg(FT_DEV_MODE_REG, buf, 1);

    return FSP_SUCCESS;
}

fsp_err_t touchpad_is_touched(void)
{
    uint8_t temp_data[16];
    uint8_t touch_p_cnt;
    fsp_err_t err;

    err = ft6336u_read_reg(FT_TD_STATUS_REG, &temp_data[0], 1);
    if (FSP_SUCCESS != err)
        return err;

    touch_p_cnt = temp_data[0] & 0x0F;

    if (touch_p_cnt > 0 && touch_p_cnt <= 2)
    {
        err = ft6336u_read_reg(FT_TP1_REG, temp_data, 12);
        if (FSP_SUCCESS != err)
            return err;

        for (int i = 0; i < TOUCH_POINT_TOTAL; i++)
        {
            if (i >= touch_p_cnt)
            {
                g_tp_drv.points_info[i].x = 0;
                g_tp_drv.points_info[i].y = 0;
                continue;
            }

            uint8_t *p_data = &temp_data[i * 6];

            uint16_t x_raw = (uint16_t)(((p_data[0] & 0x0F) << 8) | p_data[1]);
            uint16_t y_raw = (uint16_t)(((p_data[2] & 0x0F) << 8) | p_data[3]);

            g_tp_drv.points_info[i].id = (p_data[2] >> 4) & 0x0F;

            uint16_t temp;
            switch (g_tp_drv.rotation)
            {
            case TP_ROT_NONE:
                g_tp_drv.points_info[i].x = x_raw;
                g_tp_drv.points_info[i].y = y_raw;
                break;
            case TP_ROT_90:
                temp = x_raw;
                g_tp_drv.points_info[i].x = y_raw;
                g_tp_drv.points_info[i].y = g_tp_drv.height - temp;
                break;
            case TP_ROT_180:
                g_tp_drv.points_info[i].x = g_tp_drv.width - x_raw;
                g_tp_drv.points_info[i].y = g_tp_drv.height - y_raw;
                break;
            case TP_ROT_270:
                temp = x_raw;
                g_tp_drv.points_info[i].x = g_tp_drv.width - y_raw;
                g_tp_drv.points_info[i].y = temp;
                break;
            default:
                break;
            }
        }
        return FSP_SUCCESS;
    }

    return FSP_ERR_INVALID_DATA;
}

void touchpad_set_rotation(tp_rotation_t rotation)
{
    g_tp_drv.rotation = rotation;
}

void touchpad_get_pos(uint16_t *x, uint16_t *y, int index)
{
    if (index < TOUCH_POINT_TOTAL)
    {
        *x = g_tp_drv.points_info[index].x;
        *y = g_tp_drv.points_info[index].y;
    }
}

/* I2C 回调函数 */
void i2c_master2_callback(i2c_master_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (p_args->event)
    {
    case I2C_MASTER_EVENT_TX_COMPLETE:
        if (g_i2c_tx_sem != NULL)
            xSemaphoreGiveFromISR(g_i2c_tx_sem, &xHigherPriorityTaskWoken);
        break;
    case I2C_MASTER_EVENT_RX_COMPLETE:
        if (g_i2c_rx_sem != NULL)
            xSemaphoreGiveFromISR(g_i2c_rx_sem, &xHigherPriorityTaskWoken);
        break;
    default:
        /* 其他事件如 Error，也可以选择释放信号量避免超时等待 */
        break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
static fsp_err_t i2c2_wait_for_tx(void)
{
    if (xSemaphoreTake(g_i2c_tx_sem, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        return FSP_SUCCESS;
    }
    else
    {
        /* 超时了！可能是 I2C 硬件锁死或中断没触发 */
        // printf("I2C TX Timeout!\r\n");
        return FSP_ERR_TIMEOUT;
    }
}

static fsp_err_t i2c2_wait_for_rx(void)
{
    if (xSemaphoreTake(g_i2c_rx_sem, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        return FSP_SUCCESS;
    }
    else
    {
        return FSP_ERR_TIMEOUT;
    }
}

static void ft6336u_reset(void)
{
    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, TOUCH_RESET_PIN, BSP_IO_LEVEL_LOW);
    R_BSP_SoftwareDelay(20, BSP_DELAY_UNITS_MILLISECONDS);

    g_ioport.p_api->pinWrite(g_ioport.p_ctrl, TOUCH_RESET_PIN, BSP_IO_LEVEL_HIGH);
    R_BSP_SoftwareDelay(300, BSP_DELAY_UNITS_MILLISECONDS);
}

static fsp_err_t ft6336u_write_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    fsp_err_t err;
    /* 定义一个足够大的局部缓冲区，避免 malloc */
    uint8_t write_package[32];

    if (len + 1 > sizeof(write_package))
        return FSP_ERR_INVALID_SIZE;

    write_package[0] = reg;
    memcpy(&write_package[1], buf, len);

    err = g_i2c_master2.p_api->write(g_i2c_master2.p_ctrl, write_package, len + 1, false);
    if (FSP_SUCCESS != err)
        return err;

    return i2c2_wait_for_tx(); // 等待传输完成
}

static fsp_err_t ft6336u_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    fsp_err_t err;
    uint8_t reg_addr = reg;

    /* 1. 写寄存器地址 (Restart = true) */
    err = g_i2c_master2.p_api->write(g_i2c_master2.p_ctrl, &reg_addr, 1, true);
    if (FSP_SUCCESS != err)
        return err;

    err = i2c2_wait_for_tx();
    if (FSP_SUCCESS != err)
        return err; // 如果写地址失败，直接返回

    /* 2. 读数据 */
    err = g_i2c_master2.p_api->read(g_i2c_master2.p_ctrl, buf, len, false);
    if (FSP_SUCCESS != err)
        return err;

    return i2c2_wait_for_rx();
}