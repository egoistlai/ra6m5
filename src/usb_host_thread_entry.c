#include "usb_host_thread.h"
#include "ps2_gamepad.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>

/* 全局手柄数据 */
volatile ps2_gamepad_t g_ps2_data = {128, 128, 128, 128, 0, false};

/* 内部状态与缓冲区 */
static volatile bool g_usb_connected = false;

/* 缓冲区扩大到 64 字节，兼容长数据包 */
static uint8_t g_hid_report_buf[64];

/* 用于通知数据接收完毕的二值信号量 */
static SemaphoreHandle_t g_usb_read_sem = NULL;

/* 辅助函数：置位或复位 uint16_t 的某一个 bit */
static void ps2_set_bit(uint16_t *state, uint8_t state_bit, uint8_t bit)
{
    if (state_bit == 1)
    {
        *state |= (1U << bit);
    }
    else
    {
        *state &= ~(1U << bit);
    }
}

/* FSP USB 中断回调 */
void usb_host_callback(usb_event_info_t *p_event_info, usb_hdl_t cur_task, usb_onoff_t usb_state)
{
    FSP_PARAMETER_NOT_USED(cur_task);
    FSP_PARAMETER_NOT_USED(usb_state);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (p_event_info->event)
    {
    case USB_STATUS_CONFIGURED:
        g_usb_connected = true;
        g_ps2_data.is_connected = true;
        break;

    case USB_STATUS_DETACH:
        g_usb_connected = false;
        g_ps2_data.is_connected = false;
        break;

    case USB_STATUS_READ_COMPLETE:
        if (g_usb_read_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_usb_read_sem, &xHigherPriorityTaskWoken);
        }
        break;

    default:
        break;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void usb_host_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);
    fsp_err_t err;
    g_usb_read_sem = xSemaphoreCreateBinary();

    /* 硬件上电：打开 USBHS 5V 供电 */
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_00, BSP_IO_LEVEL_LOW);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 初始化 USB 主机 */
    err = R_USB_Open(&g_basic0_ctrl, &g_basic0_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("USB Host Open Failed!\r\n");
        while (1)
            vTaskDelay(100);
    }

    while (1)
    {
        if (g_usb_connected)
        {
            /* 每次请求读取 64 字节 */
            err = R_USB_Read(&g_basic0_ctrl, g_hid_report_buf, 64, 1);

            if (FSP_SUCCESS == err)
            {
                if (xSemaphoreTake(g_usb_read_sem, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    const uint8_t *data = g_hid_report_buf;
                    uint16_t key_val = 0; // 临时存储 16 位按键状态

                    /* ========================================================
                     * 【精准解码区：基于用户抓取的实际映射】
                     * ======================================================== */

                    // --- 摇杆映射 ---
                    g_ps2_data.left_x = data[3];
                    g_ps2_data.left_y = data[4];
                    g_ps2_data.right_x = data[5];
                    g_ps2_data.right_y = data[6];

                    // --- 第 1 字节 (data[0]) 按键映射 ---
                    ps2_set_bit(&key_val, (data[0] & 0x10) ? 1 : 0, 10); // 10 -> L1
                    ps2_set_bit(&key_val, (data[0] & 0x20) ? 1 : 0, 11); // 20 -> R1
                    ps2_set_bit(&key_val, (data[0] & 0x40) ? 1 : 0, 8);  // 40 -> L2
                    ps2_set_bit(&key_val, (data[0] & 0x80) ? 1 : 0, 9);  // 80 -> R2
                    ps2_set_bit(&key_val, (data[0] & 0x01) ? 1 : 0, 12); // 01 -> Y (Green)
                    ps2_set_bit(&key_val, (data[0] & 0x02) ? 1 : 0, 13); // 02 -> B (Red)
                    ps2_set_bit(&key_val, (data[0] & 0x04) ? 1 : 0, 14); // 04 -> A (Blue)
                    ps2_set_bit(&key_val, (data[0] & 0x08) ? 1 : 0, 15); // 08 -> X (Pink)

                    // --- 第 2 字节 (data[1]) 按键映射 ---
                    ps2_set_bit(&key_val, (data[1] & 0x01) ? 1 : 0, 0); // 01 -> SELECT
                    ps2_set_bit(&key_val, (data[1] & 0x02) ? 1 : 0, 3); // 02 -> START
                    ps2_set_bit(&key_val, (data[1] & 0x10) ? 1 : 0, 1); // 10 -> MODE (映射为 L3)

                    // --- 第 3 字节 (data[2]) 方向键 Hat Switch 映射 ---
                    // 标准 Hat Switch 规则：0=上, 1=右上, 2=右, 3=右下, 4=下, 5=下左, 6=左, 7=左上, 大于7为释放
                    uint8_t hat = data[2] & 0x0F;
                    if (hat > 7)
                    {
                        // 没有任何方向键按下
                        ps2_set_bit(&key_val, 0, 4); // UP
                        ps2_set_bit(&key_val, 0, 5); // RIGHT
                        ps2_set_bit(&key_val, 0, 6); // DOWN
                        ps2_set_bit(&key_val, 0, 7); // LEFT
                    }
                    else
                    {
                        switch (hat)
                        {
                        case 0:
                            ps2_set_bit(&key_val, 1, 4);
                            break; // 00—上
                        case 1:
                            ps2_set_bit(&key_val, 1, 4);
                            ps2_set_bit(&key_val, 1, 5);
                            break; // 右上
                        case 2:
                            ps2_set_bit(&key_val, 1, 5);
                            break; // 02—右
                        case 3:
                            ps2_set_bit(&key_val, 1, 6);
                            ps2_set_bit(&key_val, 1, 5);
                            break; // 右下
                        case 4:
                            ps2_set_bit(&key_val, 1, 6);
                            break; // 04—下
                        case 5:
                            ps2_set_bit(&key_val, 1, 6);
                            ps2_set_bit(&key_val, 1, 7);
                            break; // 左下
                        case 6:
                            ps2_set_bit(&key_val, 1, 7);
                            break; // 06—左
                        case 7:
                            ps2_set_bit(&key_val, 1, 4);
                            ps2_set_bit(&key_val, 1, 7);
                            break; // 左上
                        }
                    }

                    // 最后统一赋值给全局结构体，保证数据更新的原子性
                    g_ps2_data.buttons = key_val;

                    // 测试打印（确认无误后可注释掉）
                    // printf("Raw: %02X %02X %02X %02X %02X %02X %02X %02X | BTN: %04X\r\n",
                    //        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], key_val);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz 刷新率
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}