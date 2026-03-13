#ifndef _DRV_LED_H_
#define _DRV_LED_H_

/* 引入 FSP 核心头文件 */
#include "hal_data.h"
#include "SysConfig.h"

/***************************************************************************************************
 * 硬件定义适配
 * 目标：RA6M5 P400
 * 注意：通常开发板 LED 为低电平点亮 (Active Low) 或 高电平点亮。
 * 原代码逻辑：LED4_OFF ... BSRRL(Set High), LED4_ON ... BSRRH(Set Low) -> 低电平点亮
 * 如果你的板子是高电平点亮，请交换下面的 LOW 和 HIGH
 **************************************************************************************************/
#define LED_PIN_P400 BSP_IO_PORT_04_PIN_00

/* 定义开关动作 (假设低电平点亮) */
#define LED_ON_LEVEL BSP_IO_LEVEL_LOW
#define LED_OFF_LEVEL BSP_IO_LEVEL_HIGH

/* * 映射 LED1 到 P400
 * 使用 FSP 的 R_IOPORT_PinWrite 进行操作
 */
#define LED1_ON R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_P400, LED_ON_LEVEL)
#define LED1_OFF R_IOPORT_PinWrite(&g_ioport_ctrl, LED_PIN_P400, LED_OFF_LEVEL)

/* * 其他 LED (LED2, LED3, LED4) 没有物理引脚
 * 定义为空宏，防止飞控代码报错
 */
#define LED2_ON (void)0
#define LED2_OFF (void)0
#define LED3_ON (void)0
#define LED3_OFF (void)0
#define LED4_ON (void)0
#define LED4_OFF (void)0

/* 保持原有的位定义，供逻辑层使用 */
#define LED_R_BIT 0x01
#define LED_G_BIT 0x02
#define LED_B_BIT 0x04
#define LED_A_BIT 0x08
#define LED_ALL_BIT 0x0f

typedef union
{
    //
    s8 brightness[LED_NUM];

} _led_st;

extern _led_st led;

/***************************************************************************************************
 * 函数声明
 **************************************************************************************************/
/* 保持函数名不变，方便兼容飞控原有调用 */
void DvrLedInit(void);
void LED_On_Off(uint16_t leds);
void LED_1ms_DRV(void);

#endif /* _DRV_LED_H_ */