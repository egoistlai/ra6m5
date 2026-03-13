/***************************************************************************************************
 * Includes
 **************************************************************************************************/
#include "Drv_RcIn.h"
#include "ANO_LX.h" // 引用飞控核心头文件，用于调用解析函数
#include "hal_data.h"
#include "Drv_BSP.h"

/***************************************************************************************************
 * External Variables
 **************************************************************************************************/
/* 引用 FSP 生成的 SCI2 实例 */
extern const uart_instance_t g_uart2;

/***************************************************************************************************
 * Functions
 **************************************************************************************************/

/**
 * @brief  SBUS初始化函数
 * 适配 RA6M5 SCI2
 */
void DrvRcSbusInit(void)
{
    fsp_err_t err;

    /* 1. 打开串口 (默认配置) */
    err = g_uart2.p_api->open(g_uart2.p_ctrl, g_uart2.p_cfg);
    if (FSP_SUCCESS != err)
        __BKPT();

    /* ============================================================ */
    /* 2. 【关键步骤】手动开启硬件反相 (SINV bit) */
    /* 这会将 High/Low 电平逻辑互换，适配 SBUS */
    /* ============================================================ */

    /* 获取寄存器基地址 (强转为 SCI 寄存器结构体) */
    R_SCI0_Type *p_sci_reg = (R_SCI0_Type *)((sci_uart_instance_ctrl_t *)g_uart2.p_ctrl)->p_reg;

    /* 必须先关闭 TE/RE 才能修改 SCMR，但 open 后通常已经开启了。
       为了安全，我们先暂停，改完再恢复。 */
    uint8_t original_scr = p_sci_reg->SCR; // 保存当前中断/使能状态

    p_sci_reg->SCR = 0; // 暂时关闭串口逻辑

    /* 修改 SCMR 寄存器的 SINV 位 (Bit 2) */
    /* SINV = 1: Inverted data (Low is logic 1) -> 适配 SBUS */
    p_sci_reg->SCMR_b.SINV = 1;

    /* 恢复之前的设置 (重新开启 RE/RIE) */
    p_sci_reg->SCR = original_scr;

    /* ============================================================ */
}

/**
 * @brief  SCI2 中断回调函数
 * 负责接收 SBUS 字节流并喂给 ANO 解析器
 */
void sbus_callback(uart_callback_args_t *p_args)
{
    /* 处理接收字符事件 */
    if (UART_EVENT_RX_CHAR == p_args->event)
    {
        /* 获取接收到的一个字节 */
        uint8_t rcv_data = (uint8_t)p_args->data;

        /* 调用 ANO 飞控的 SBUS 解析状态机 */
        /* 这个函数通常在 ANO_LX.c 或 ANO_DT_LX.c 中定义 */
        DrvSbusGetOneByte(rcv_data);
    }

    /* 处理错误事件 (可选) */
    else if (UART_EVENT_ERR_PARITY == p_args->event ||
             UART_EVENT_ERR_FRAMING == p_args->event ||
             UART_EVENT_ERR_OVERFLOW == p_args->event)
    {
        /* SBUS 信号在插拔时容易产生错误，通常只需忽略即可，
           解析层有校验机制会自动丢弃错误包 */
    }
}