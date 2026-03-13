#ifndef __DRV_RC_INPUT_H
#define __DRV_RC_INPUT_H

#include "hal_data.h"

/***************************************************************************************************
 * 导出函数
 **************************************************************************************************/
/* 只保留 SBUS 初始化 */
void DrvRcSbusInit(void);

/* * SBUS 数据解析回调
 * 注意：需要在 FSP 配置中将 UART2 的 Callback 设置为 "sbus_callback"
 */
void sbus_callback(uart_callback_args_t *p_args);

#endif /* __DRV_RC_INPUT_H */