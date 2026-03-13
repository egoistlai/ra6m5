/***************************************************************************************************
 * Includes
 **************************************************************************************************/
#include "Drv_adc.h"
#include "FreeRTOS.h"
#include "task.h"

/***************************************************************************************************
 * Variables
 **************************************************************************************************/

/* 扫描完成标志位 */
static volatile bool scan_complete_flag = false;

/***************************************************************************************************
 * Functions
 **************************************************************************************************/

/* * ADC 回调函数
 * 需要在 FSP 中将 g_adc0 的 Callback 设置为 "adc0_callback"
 */
void adc0_callback(adc_callback_args_t *p_args)
{
    if (ADC_EVENT_SCAN_COMPLETE == p_args->event)
    {
        scan_complete_flag = true;
    }
}

/*
 * 初始化函数
 */
fsp_err_t DrvAdcInit(void)
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

/*
 * 获取电池电压 (带软件滤波)
 * 替代了原 STM32 代码中 DMA 自动填充 buffer 后取平均的逻辑
 */
float Drv_AdcGetBatVot(void)
{
    fsp_err_t err;
    uint16_t adc_raw_data;
    uint32_t sum = 0;
    const uint8_t sample_count = 20; // 采样 20 次求平均，足够稳定

    /* 循环采样 */
    for (int i = 0; i < sample_count; i++)
    {
        scan_complete_flag = false;

        /* 1. 启动扫描 */
        err = R_ADC_ScanStart(&g_adc0_ctrl);
        if (FSP_SUCCESS != err)
            return 0.0f;

        /* 2. 等待完成 (使用简单的超时等待) */
        /* 由于 ADC 极快 (几微秒)，这里可以用忙等待，不会明显阻塞 RTOS */
        uint32_t timeout = 10000;
        while (!scan_complete_flag && timeout > 0)
        {
            timeout--;
        }

        err = R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_1, &adc_raw_data);

        if (FSP_SUCCESS == err)
        {
            sum += adc_raw_data;
        }
    }

    /* 计算平均值 */
    float avg_adc = (float)sum / (float)sample_count;

    /* 转换为电压值 */
    /* 公式：ADC值 * (3.3 / 4096) * 分压系数 */
    float voltage = avg_adc * (ADC_REF_VOLTAGE / ADC_PRECISION) * ADC_DIVIDER_RATIO;

    return voltage;
}