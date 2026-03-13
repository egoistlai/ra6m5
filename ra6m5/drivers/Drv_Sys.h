#ifndef _DRV_SYS_H_
#define _DRV_SYS_H_

/* 引入 FSP 和 标准头文件 */
#include "hal_data.h"
#include "SysConfig.h"

/* 定义常用类型简写，保持与 ANO 代码兼容 */

/***************************************************************************************************
 * 导出函数
 **************************************************************************************************/
void DrvSysInit(void);

/* 获取系统运行时间 (毫秒) - 基于 FreeRTOS Tick */
uint32_t GetSysRunTimeMs(void);

/* 获取系统运行时间 (微秒) - 基于 FreeRTOS Tick + SysTick 偏差值 */
uint32_t GetSysRunTimeUs(void);

/* 毫秒延时 (智能切换：调度器开启时挂起任务，未开启时死循环) */
void MyDelayMs(u32 time);

/* 微秒延时 (死循环，基于 DWT 硬件计数器) */
void MyDelayUs(u32 time);

#endif /* _DRV_SYS_H_ */