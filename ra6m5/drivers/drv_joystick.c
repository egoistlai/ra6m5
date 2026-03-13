#include "drv_joystick.h"
#include <stdio.h>

/* ADC阈值定义 (12位ADC: 0~4095) */
/* 假设摇杆供电和MCU电平一致。静止约2048 */
#define ADC_THRESHOLD_LOW 1000  // 小于此值认为向左或向下
#define ADC_THRESHOLD_HIGH 3800 // 大于此值认为向右或向上

volatile bool scan_complete_flag = false;
void adc_callback(adc_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    scan_complete_flag = true;
}

fsp_err_t drv_joystick_init(void)
{
    fsp_err_t err;

    /* 1. 初始化 ADC */
    err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
    if (FSP_SUCCESS != err)
        return err;

    /* 2. 配置扫描通道 (Ch0 + Ch1) */
    err = R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc0_channel_cfg);
    if (FSP_SUCCESS != err)
        return err;

    return FSP_SUCCESS;
}

joy_dir_t drv_joystick_get_dir(void)
{
    uint16_t adc_data_x = 0;
    uint16_t adc_data_y = 0;
    volatile uint32_t timeout = 10000;

        /* 1. 启动一次扫描 */
    R_ADC_ScanStart(&g_adc0_ctrl);
    while (!scan_complete_flag && timeout > 0)
    {
        timeout--;
    }

    if (timeout == 0)
    {
        /* 如果超时了，直接返回无动作，避免卡死 */
        return JOY_CENTER;
    }
    scan_complete_flag = false;

    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_1, &adc_data_x);
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_3, &adc_data_y);

    // printf("data_x: %d, data_y: %d \r\n", adc_data_x, adc_data_y);

    /* 4. 判断方向 */
    /* 注意：根据你的安装方向，X/Y 和 上下左右的对应关系可能需要调整 */

    // 判断 X 轴
    if (adc_data_x < ADC_THRESHOLD_LOW)
    {
        return JOY_LEFT;
    }
    else if (adc_data_x > ADC_THRESHOLD_HIGH)
    {
        return JOY_RIGHT;
    }

    // 判断 Y 轴
    if (adc_data_y < ADC_THRESHOLD_LOW)
    {
        return JOY_UP;
    }
    else if (adc_data_y > ADC_THRESHOLD_HIGH)
    {
        return JOY_DOWN;
    }

    return JOY_CENTER; // 在中间区域
}
