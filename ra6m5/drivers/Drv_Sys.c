/***************************************************************************************************
 * Includes
 **************************************************************************************************/
#include "Drv_Sys.h"
#include "FreeRTOS.h"
#include "task.h"

/***************************************************************************************************
 * Functions
 **************************************************************************************************/

/**
 * @brief  系统计时初始化
 * 在 FreeRTOS 中 SysTick 已自动配置，这里主要开启 DWT 计数器用于微秒延时
 */
void DrvSysInit(void)
{
    /* 解锁 DWT (Data Watchpoint and Trace) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 清零循环计数器 */
    DWT->CYCCNT = 0;

    /* 开启循环计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  获取系统运行毫秒数
 * 直接返回 FreeRTOS 的 Tick 计数
 */
uint32_t GetSysRunTimeMs(void)
{
    /* 如果在中断中调用，需要使用 FromISR 版本 */
    if (xPortIsInsideInterrupt())
    {
        return xTaskGetTickCountFromISR();
    }
    else
    {
        return xTaskGetTickCount();
    }
}

/**
 * @brief  获取系统运行微秒数
 * 原理：Tick数 * 1000 + 当前 Tick 逝去的时间(由 SysTick 寄存器计算)
 */
uint32_t GetSysRunTimeUs(void)
{
    uint32_t tick_ms;
    uint32_t systick_val;
    uint32_t systick_load;
    uint32_t us_in_current_tick;

    /* 1. 获取当前的 ms 计数 */
    tick_ms = GetSysRunTimeMs();

    /* 2. 获取 SysTick 当前计数值 (注意：SysTick 是向下计数的) */
    systick_val = SysTick->VAL;
    systick_load = SysTick->LOAD;

    /* 3. 计算当前 Tick 已经过去的时钟周期数 */
    /* elapsed_cycles = LOAD - VAL */
    uint32_t elapsed_cycles = systick_load - systick_val;

    /* 4. 将周期数转换为微秒 */
    /* SystemCoreClock 是系统主频 (Hz)。除以 1M 得到 1微秒的周期数 */
    /* 例如 200MHz 主频，1us = 200 个周期 */
    us_in_current_tick = elapsed_cycles / (SystemCoreClock / 1000000);

    /* 5. 合成总微秒数 */
    return (tick_ms * 1000 + us_in_current_tick);
}

/**
 * @brief  微秒级延时 (阻塞死循环)
 * 使用 DWT->CYCCNT 硬件寄存器，不依赖 RTOS 调度，非常精准
 */
void MyDelayUs(uint32_t us)
{
    uint32_t start_cycles = DWT->CYCCNT;

    /* 计算需要等待的时钟周期数 */
    /* us * (SystemCoreClock / 1000000) */
    /* 为了防止乘法溢出，建议 SystemCoreClock/1000000 预计算，RA6M5 200MHz -> 200 */
    uint32_t wait_cycles = us * (SystemCoreClock / 1000000);

    /* 循环等待，利用 32位无符号整数溢出特性，无需担心 CYCCNT 翻转 */
    while ((DWT->CYCCNT - start_cycles) < wait_cycles)
        ;
}

/**
 * @brief  毫秒级延时
 * 智能切换：如果调度器已启动，使用 vTaskDelay 释放 CPU；
 * 如果还在初始化阶段(调度器未启动)，使用死循环等待。
 */
void MyDelayMs(u32 time)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        /* 任务运行中，使用 OS 延时，允许其他任务运行 */
        vTaskDelay(pdMS_TO_TICKS(time));
    }
    else
    {
        /* 初始化阶段，OS 还没跑起来，使用 FSP 自带的软件忙等待 */
        R_BSP_SoftwareDelay(time, BSP_DELAY_UNITS_MILLISECONDS);
    }
}