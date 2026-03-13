#ifndef DRV_UART_H
#define DRV_UART_H

/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "hal_data.h"
#include <stdio.h>

/***********************************************************************************************************************
 * Exported global functions
 **********************************************************************************************************************/

/* --- UART7: 飞控/数传 --- */
fsp_err_t drv_uart7_init(void);
void DrvUart7SendBuf(unsigned char *DataToSend, uint8_t data_num);
void uart7_callback(uart_callback_args_t *p_args);

/* --- UART2: 光流模块 (Optical Flow) --- */
/* 新增：完全对齐 UART7 的接口规范 */
fsp_err_t drv_uart2_init(void);
void DrvUart2SendBuf(unsigned char *DataToSend, uint8_t data_num);
void uart2_callback(uart_callback_args_t *p_args);

/* --- UART3: 激光雷达 (Lidar) --- */
/* 新增：完全对齐 UART7 的接口规范 */
fsp_err_t drv_uart3_init(void);
void DrvUart3SendBuf(unsigned char *DataToSend, uint8_t data_num);
void uart3_callback(uart_callback_args_t *p_args);

/* --- UART6: W800 WiFi --- */
fsp_err_t drv_uart6_init(void);
void DrvUart6SendBuf(unsigned char *DataToSend, uint8_t data_num);
void uart6_callback(uart_callback_args_t *p_args);

void uart5_callback(uart_callback_args_t *p_args);
void drv_uart5_init(void);

/* 兼容旧代码的宏 */
#define UartSendLXIMU DrvUart7SendBuf
#define drv_uart_init(void) drv_uart7_init(void)

#endif /* DRV_UART_H */