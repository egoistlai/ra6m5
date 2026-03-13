/***************************************************************************************************
 * Includes
 **************************************************************************************************/
#include "Drv_led.h"

/***************************************************************************************************
 * Functions
 **************************************************************************************************/

/* * LED 初始化函数
 * 在任务开始前调用此函数
 */
void DvrLedInit(void)
{
    fsp_err_t err;

    /* * 配置 P400 为 GPIO 输出模式
     * 虽然通常在 FSP Configuration 图形界面中配置，但代码里强制配置更保险
     * 初始状态设为 OFF (High)
     */
    err = R_IOPORT_PinCfg(&g_ioport_ctrl, LED_PIN_P400,
                          IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH);

    /* 确保初始状态为灭 */
    LED1_OFF;

    if (FSP_SUCCESS != err)
    {
        /* 初始化失败处理 (可选) */
        // __BKPT(0);
    }
}

/* * 统一控制函数
 * 飞控代码通过传入位掩码来控制多个 LED
 * 我们只响应 LED_R_BIT (LED1)
 */
void LED_On_Off(uint16_t leds)
{
    /* 处理 LED1 (映射到 P400) */
    if (leds & LED_R_BIT)
    {
        LED1_ON;
    }
    else
    {
        LED1_OFF;
    }

    /* * LED2, LED3, LED4 的逻辑被忽略，因为没有物理 LED
     * 如果你希望看到所有状态，可以将下面的条件改为 if (leds & (LED_G_BIT | ...)) LED1_ON;
     * 但这会导致 LED 闪烁混乱，建议目前先只看一个状态。
     */

    /* if (leds & LED_G_BIT) LED2_ON; else LED2_OFF;
    if (leds & LED_B_BIT) LED3_ON; else LED3_OFF;
    if (leds & LED_A_BIT) LED4_ON; else LED4_OFF;
    */
}

_led_st led;

void LED_1ms_DRV(void)
{
    static u16 led_cnt[LED_NUM];
    u16 led_tmp;
    for (u8 i = 0; i < LED_NUM; i++)
    {

        if (led_cnt[i] < (s16)led.brightness[i])
        {
            // ON
            led_tmp |= (1 << i);
        }
        else
        {
            // OFF
            led_tmp &= ~(1 << i);
        }

        if (++led_cnt[i] >= 20)
        {
            led_cnt[i] = 0;
        }
    }
    //
    LED_On_Off(led_tmp);
}